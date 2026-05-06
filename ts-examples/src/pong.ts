import { Font, Renderer } from 'renderer';
import { Collection, Rectangle } from 'shapes';
import { Format } from './constants.js';
import * as adc from "adc";
import { buildModesetBuffer, buildSyncBuffer, sendRpHub75Frame, setupSpi } from './spiSender.js';

// --- CONFIGURATION ---
const PANEL_WIDTH = 64;
const PANEL_HEIGHT = 64;

// --- HARDWARE CONFIG ---
// Using the Y-axes from your dual-stick setup
const P1_Y_PIN = 6; // Left Stick Y
const P2_Y_PIN = 5; // Right Stick Y

adc.configure(P1_Y_PIN);
adc.configure(P2_Y_PIN);

const CENTER = 512;
const DEADZONE = 200;

// --- GAME CONSTANTS ---
const PADDLE_W = 2;
const PADDLE_H = 12;
const PADDLE_MARGIN = 2;
const BALL_SIZE = 2;
const MAX_BALL_SPEED = 4;
const INITIAL_BALL_SPEED = 1.5;

const font = new Font();

// --- GAME STATE ---
let p1 = { y: 26, score: 0, speed: 2.5 };
let p2 = { y: 26, score: 0, speed: 2.5 };
let ball = { x: 31, y: 31, vx: INITIAL_BALL_SPEED, vy: 0.5 };

// --- INPUT UTILITY ---
function getJoystickDirection(pin) {
    const raw = adc.read(pin) - CENTER;
    if (Math.abs(raw) < DEADZONE) return 0;
    // Normalize to 1 or -1 based on direction
    return raw > 0 ? 1 : -1;
}

// --- GAME LOGIC ---
function resetBall(scorer) {
    ball.x = Math.floor(PANEL_WIDTH / 2) - Math.floor(BALL_SIZE / 2);
    ball.y = Math.floor(PANEL_HEIGHT / 2) - Math.floor(BALL_SIZE / 2);
    // Serve towards the player who just got scored on
    ball.vx = (scorer === 1 ? -INITIAL_BALL_SPEED : INITIAL_BALL_SPEED);
    ball.vy = (Math.random() - 0.5) * 2;
}

function constrainPaddle(player) {
    if (player.y < 0) player.y = 0;
    if (player.y > PANEL_HEIGHT - PADDLE_H) player.y = PANEL_HEIGHT - PADDLE_H;
}

function checkPaddleCollision(paddleX, paddleY, isLeftPaddle) {
    // Basic AABB Collision
    if (
        ball.x < paddleX + PADDLE_W &&
        ball.x + BALL_SIZE > paddleX &&
        ball.y < paddleY + PADDLE_H &&
        ball.y + BALL_SIZE > paddleY
    ) {
        // Correct position to prevent getting stuck inside the paddle
        ball.x = isLeftPaddle ? paddleX + PADDLE_W : paddleX - BALL_SIZE;

        // Reverse X velocity and increase speed slightly
        ball.vx = -ball.vx * 1.1;

        // Cap maximum speed
        if (ball.vx > MAX_BALL_SPEED) ball.vx = MAX_BALL_SPEED;
        if (ball.vx < -MAX_BALL_SPEED) ball.vx = -MAX_BALL_SPEED;

        // Add "English" (Vertical deflection based on hit position relative to paddle center)
        const paddleCenter = paddleY + (PADDLE_H / 2);
        const ballCenter = ball.y + (BALL_SIZE / 2);
        const hitFactor = (ballCenter - paddleCenter) / (PADDLE_H / 2);
        ball.vy += hitFactor * 1.5;
    }
}

// --- MAIN LOOP ---
export async function runPong() {
    setupSpi();
    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT);
    const renderBuffer = new ArrayBuffer(PANEL_WIDTH * PANEL_HEIGHT * 3);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_888);

    const POLL_INTERVAL = 16; // Approx 60fps

    while (true) {
        // --- 1. Input & Update ---
        const p1Move = getJoystickDirection(P1_Y_PIN);
        const p2Move = getJoystickDirection(P2_Y_PIN);

        p1.y -= p1Move * p1.speed;
        p2.y += p2Move * p2.speed;

        constrainPaddle(p1);
        constrainPaddle(p2);

        // Update Ball
        ball.x += ball.vx;
        ball.y += ball.vy;

        // Wall Collisions (Top & Bottom)
        if (ball.y <= 0) {
            ball.y = 0;
            ball.vy *= -1;
        } else if (ball.y >= PANEL_HEIGHT - BALL_SIZE) {
            ball.y = PANEL_HEIGHT - BALL_SIZE;
            ball.vy *= -1;
        }

        // Paddle Collisions
        checkPaddleCollision(PADDLE_MARGIN, p1.y, true);
        checkPaddleCollision(PANEL_WIDTH - PADDLE_MARGIN - PADDLE_W, p2.y, false);

        // Scoring
        if (ball.x < 0) {
            p2.score++;
            resetBall(2);
        } else if (ball.x > PANEL_WIDTH) {
            p1.score++;
            resetBall(1);
        }

        // --- 2. Render Scene ---
        const scene = new Collection({ x: 0, y: 0, color: [0, 0, 0, 255] }); // Black background

        // Draw Center Net (Dotted Line)
        for (let i = 0; i < PANEL_HEIGHT; i += 4) {
            scene.add(new Rectangle({
                x: Math.floor(PANEL_WIDTH / 2) - 1,
                y: i,
                width: 1,
                height: 2,
                color: [100, 100, 100, 255],
                fill: true
            }));
        }

        // Draw P1 Paddle (Blue)
        scene.add(new Rectangle({
            x: PADDLE_MARGIN,
            y: p1.y,
            width: PADDLE_W,
            height: PADDLE_H,
            color: [50, 150, 255, 255],
            fill: true
        }));

        // Draw P2 Paddle (Red)
        scene.add(new Rectangle({
            x: PANEL_WIDTH - PADDLE_MARGIN - PADDLE_W,
            y: p2.y,
            width: PADDLE_W,
            height: PADDLE_H,
            color: [255, 50, 50, 255],
            fill: true
        }));

        // Draw Ball (White)
        scene.add(new Rectangle({
            x: ball.x,
            y: ball.y,
            width: BALL_SIZE,
            height: BALL_SIZE,
            color: [255, 255, 255, 255],
            fill: true
        }));

        // Render shapes to buffer
        renderer.render(scene, renderBuffer, true, Format.RGB_888, -1);

        // Draw Scores
        renderer.drawText(
            renderBuffer,
            p1.score.toString(),
            Math.floor(PANEL_WIDTH / 4), 2,
            font,
            [50, 150, 255, 255],
            false,
            Format.RGB_888,
            -1
        );

        renderer.drawText(
            renderBuffer,
            p2.score.toString(),
            Math.floor((PANEL_WIDTH / 4) * 3) - 4, 2,
            font,
            [255, 50, 50, 255],
            false,
            Format.RGB_888,
            -1
        );

        // --- 3. Push to Display & Sleep ---
        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);

        await sleep(POLL_INTERVAL);
    }
}

runPong();
