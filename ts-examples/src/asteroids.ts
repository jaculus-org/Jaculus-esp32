import { Renderer } from 'renderer';
import { Collection, Polygon, Circle, RegularPolygon } from 'shapes';
import { Format } from './constants.js';
import * as adc from "adc";
import { buildModesetBuffer, buildSyncBuffer, sendRpHub75Frame, setupSpi } from './spiSender.js';

// --- CONFIGURATION ---
const PANEL_WIDTH = 64;
const PANEL_HEIGHT = 64;

// --- HARDWARE CONFIG ---
const MOVE_X = 4;
const MOVE_Y = 5;
const AIM_X = 7;
const AIM_Y = 6;

adc.configure(MOVE_X);
adc.configure(MOVE_Y);
adc.configure(AIM_X);
adc.configure(AIM_Y);

const CENTER = 512;
const DEADZONE = 200;

// --- INPUT UTILITIES ---
function getStickVector(pinX, pinY, swapped) {
    const rawX = adc.read(pinX) - CENTER;
    const rawY = adc.read(pinY) - CENTER;

    if (Math.abs(rawX) < DEADZONE && Math.abs(rawY) < DEADZONE) {
        return { x: 0, y: 0, active: false };
    }

    // Normalize the vector so diagonal movement isn't faster
    const mag = Math.sqrt(rawX * rawX + rawY * rawY);
    return {
        x: (swapped ? -1 : 1) * rawX / mag,
        y: (swapped ? -1 : 1) * rawY / mag,
        active: true
    };
}

// --- GAME LOGIC ---
let player = { x: 32, y: 32, angle: 0, speed: 2 };
let bullets = [];
let asteroids = [];
let fireCooldown = 0;

function spawnAsteroid() {
    // Spawn randomly on the edges
    const edge = Math.floor(Math.random() * 4);
    let ax, ay;
    if (edge === 0) { ax = Math.random() * 64; ay = 0; }
    else if (edge === 1) { ax = 64; ay = Math.random() * 64; }
    else if (edge === 2) { ax = Math.random() * 64; ay = 64; }
    else { ax = 0; ay = Math.random() * 64; }

    asteroids.push({
        x: ax,
        y: ay,
        vx: (Math.random() - 0.5) * 1.5,
        vy: (Math.random() - 0.5) * 1.5,
        r: 4 + Math.random() * 4
    });
}

function resetGame() {
    player = { x: 32, y: 32, angle: 0, speed: 2 };
    bullets = [];
    asteroids = [];
    for (let i = 0; i < 4; i++) spawnAsteroid();
}

export async function runAsteroids() {
    setupSpi();
    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT);
    const renderBuffer = new ArrayBuffer(PANEL_WIDTH * PANEL_HEIGHT * 3);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_888);

    const FRAME_TIME = 1;
    const POLL_INTERVAL = 1;

    resetGame();

    while (true) {
        let moveInput = { x: 0, y: 0, active: false };
        let aimInput = { x: 0, y: 0, active: false };

        // --- 1. Responsive Input Polling ---
        let elapsed = 0;
        while (elapsed < FRAME_TIME) {
            let m = getStickVector(MOVE_X, MOVE_Y, false);
            let a = getStickVector(AIM_X, AIM_Y, true);
            // Latch the input if it was active at any point during polling
            if (m.active) moveInput = m;
            if (a.active) aimInput = a;

            await sleep(POLL_INTERVAL);
            elapsed += POLL_INTERVAL;
        }

        // --- 2. Update Logic ---

        // Player Movement
        if (moveInput.active) {
            player.x += moveInput.x * player.speed;
            player.y += moveInput.y * player.speed;

            player.angle = Math.atan2(-moveInput.y, moveInput.x) * 180 / Math.PI;
        }

        // Player Aim & Shooting
        if (aimInput.active) {
            player.angle = Math.atan2(-aimInput.y, aimInput.x) * 180 / Math.PI;

            if (fireCooldown <= 0) {
                bullets.push({
                    x: player.x,
                    y: player.y,
                    vx: aimInput.x * 5,
                    vy: aimInput.y * 5,
                    life: 20 // Frames until bullet despawns
                });
                fireCooldown = 6; // Fire rate delay
            }
        }
        if (fireCooldown > 0) fireCooldown--;

        // Screen Wrapping (Player)
        player.x = (player.x + PANEL_WIDTH) % PANEL_WIDTH;
        player.y = (player.y + PANEL_HEIGHT) % PANEL_HEIGHT;

        // Update Bullets
        for (let i = bullets.length - 1; i >= 0; i--) {
            let b = bullets[i];
            b.x += b.vx;
            b.y += b.vy;
            b.life--;

            // Screen Wrapping (Bullets)
            // b.x = (b.x + PANEL_WIDTH) % PANEL_WIDTH;
            // b.y = (b.y + PANEL_HEIGHT) % PANEL_HEIGHT;
            if (b.x < 0 || b.x >= PANEL_WIDTH || b.y < 0 || b.y >= PANEL_HEIGHT) {
                b.life = 0;
            }

            if (b.life <= 0) bullets.splice(i, 1);
        }

        // Update Asteroids & Collisions
        for (let i = asteroids.length - 1; i >= 0; i--) {
            let a = asteroids[i];
            a.x += a.vx;
            a.y += a.vy;

            // Screen Wrapping (Asteroids)
            a.x = (a.x + PANEL_WIDTH) % PANEL_WIDTH;
            a.y = (a.y + PANEL_HEIGHT) % PANEL_HEIGHT;

            // Player Collision (Death)
            const dxP = player.x - a.x;
            const dyP = player.y - a.y;
            if (Math.sqrt(dxP * dxP + dyP * dyP) < a.r + 2) {
                resetGame();
                break;
            }

            // Bullet Collision
            for (let j = bullets.length - 1; j >= 0; j--) {
                let b = bullets[j];
                const dx = b.x - a.x;
                const dy = b.y - a.y;
                if (Math.sqrt(dx * dx + dy * dy) < a.r + 2) {
                    asteroids.splice(i, 1);
                    bullets.splice(j, 1);
                    spawnAsteroid(); // Spawn a new one to keep the game going
                    break;
                }
            }
        }

        // --- 3. Render Frame ---
        const scene = new Collection({ x: 0, y: 0, color: [0, 0, 0, 255] });

        // Draw Asteroids (Hexagons)
        for (let a of asteroids) {
            scene.add(new RegularPolygon({
                x: a.x, y: a.y,
                sides: 6, radius: a.r,
                color: [100, 100, 255, 255], // Light Blue
                fill: false,
            }));
        }

        // Draw Bullets (Tiny circles)
        for (let b of bullets) {
            scene.add(new Circle({
                x: b.x, y: b.y,
                radius: 1,
                color: [255, 255, 0, 255], // Yellow
                fill: true
            }));
        }

        // Draw Player (Triangle/Polygon)
        // A simple triangle pointing "Right" by default
        const ship = new Polygon({
            x: player.x,
            y: player.y,
            vertices: [[3, 0], [-2, -2], [-2, 2]],
            color: [0, 255, 0, 255], // Green
            fill: true
        });

        ship.setRotationAngle(player.angle);
        scene.add(ship);

        // Push to display
        renderer.render(scene, renderBuffer, true, Format.RGB_888, -1);
        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);
    }
}

runAsteroids();
