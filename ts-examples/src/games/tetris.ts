import { Font, Renderer } from 'renderer';
import { Collection, Rectangle } from 'shapes';
import { Format } from '../constants.js';
import * as adc from "adc";
import * as gpio from "gpio";
import { buildModesetBuffer, buildSyncBuffer, sendRpHub75Frame, setupSpi } from '../spiSender.js';

// --- CONFIGURATION ---
const PANEL_WIDTH = 64;
const PANEL_HEIGHT = 64;

const COLS = 10;
const ROWS = 20;
const CELL_SIZE = 3;
const OFFSET_X = Math.floor((PANEL_WIDTH - (COLS * CELL_SIZE)) / 2);
const OFFSET_Y = Math.floor((PANEL_HEIGHT - (ROWS * CELL_SIZE)) / 2);

const ADC_X = 4;
const ADC_Y = 5;
adc.configure(ADC_X);
adc.configure(ADC_Y);

const DEADZONE = 300;

// --- GAME DATA ---
const TETROMINOES: { shape: number[][], color: [number, number, number, number] }[] = [
    { shape: [[1, 1, 1, 1]], color: [0, 255, 255, 255] },         // I - Cyan
    { shape: [[1, 0, 0], [1, 1, 1]], color: [0, 0, 255, 255] },   // J - Blue
    { shape: [[0, 0, 1], [1, 1, 1]], color: [255, 165, 0, 255] }, // L - Orange
    { shape: [[1, 1], [1, 1]], color: [255, 255, 0, 255] },       // O - Yellow
    { shape: [[0, 1, 1], [1, 1, 0]], color: [0, 255, 0, 255] },   // S - Green
    { shape: [[0, 1, 0], [1, 1, 1]], color: [128, 0, 128, 255] }, // T - Purple
    { shape: [[1, 1, 0], [0, 1, 1]], color: [255, 0, 0, 255] }    // Z - Red
];

const font = new Font();

let board = Array.from({ length: ROWS }, () => Array(COLS).fill(null));
let currentPiece = null;

let nextPieceTemplate = TETROMINOES[Math.floor(Math.random() * TETROMINOES.length)];

const joyState = { left: false, right: false, up: false, down: false };
const lastJoyState = { left: false, right: false, up: false, down: false };

let ghostY = 0;
let needsRender = true;

// --- GAME LOGIC ---

function updateJoystickState(state) {
    const xVal = adc.read(ADC_X);
    const yVal = adc.read(ADC_Y);
    const dx = xVal - 512;
    const dy = yVal - 512;

    state.left = false;
    state.right = false;
    state.up = false;
    state.down = false;

    if (Math.abs(dx) > DEADZONE || Math.abs(dy) > DEADZONE) {
        if (Math.abs(dx) > Math.abs(dy)) {
            if (dx > 0) state.right = true;
            if (dx < 0) state.left = true;
        } else {
            if (dy > 0) state.down = true;
            if (dy < 0) state.up = true;
        }
    }
}

function updateGhostY() {
    ghostY = currentPiece.y;
    while (!checkCollision(currentPiece, currentPiece.x, ghostY + 1)) {
        ghostY++;
    }
}

function spawnPiece() {
    const template = nextPieceTemplate;
    const c = template.color;

    currentPiece = {
        shape: template.shape,
        color: c,
        ghostColor: [Math.floor(c[0] * 0.4), Math.floor(c[1] * 0.4), Math.floor(c[2] * 0.4), 255],
        x: Math.floor(COLS / 2) - Math.floor(template.shape[0].length / 2),
        y: 0
    };

    nextPieceTemplate = TETROMINOES[Math.floor(Math.random() * TETROMINOES.length)];

    updateGhostY();
    needsRender = true;
}

function rotateMatrix(matrix) {
    return matrix[0].map((val, index) => matrix.map(row => row[index]).reverse());
}

function checkCollision(piece, targetX, targetY) {
    for (let y = 0; y < piece.shape.length; y++) {
        for (let x = 0; x < piece.shape[y].length; x++) {
            if (piece.shape[y][x]) {
                const newX = targetX + x;
                const newY = targetY + y;

                if (newX < 0 || newX >= COLS || newY >= ROWS) return true;
                if (newY >= 0 && board[newY][newX] !== null) return true;
            }
        }
    }
    return false;
}

function mergePiece() {
    for (let y = 0; y < currentPiece.shape.length; y++) {
        for (let x = 0; x < currentPiece.shape[y].length; x++) {
            if (currentPiece.shape[y][x]) {
                const boardY = currentPiece.y + y;
                if (boardY >= 0) {
                    board[boardY][currentPiece.x + x] = currentPiece.color;
                }
            }
        }
    }
}

function clearLines() {
    let linesCleared = 0;
    for (let y = ROWS - 1; y >= 0; y--) {
        if (board[y].every(cell => cell !== null)) {
            board.splice(y, 1);
            board.unshift(Array(COLS).fill(null));
            y++;
            linesCleared++;
        }
    }
    return linesCleared;
}

export async function runTetris(startSpi: boolean) {
    if (startSpi) { setupSpi(); }
    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT);
    const renderBuffer = new ArrayBuffer(PANEL_WIDTH * PANEL_HEIGHT * 2);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_565_LITTLE);

    const POLL_INTERVAL = 8;
    const DAS_DELAY = 10;
    const ARR_RATE = 10;

    const START_SPEED = 30;
    let dropInterval = START_SPEED;
    let dropCounter = 0;
    let dasTimer = 0;
    let score = 0;

    spawnPiece();

    while (gpio.read(7)) {
        // --- 1. Input & Update Logic ---
        updateJoystickState(joyState);

        if (joyState.left) {
            if (!lastJoyState.left) {
                if (!checkCollision(currentPiece, currentPiece.x - 1, currentPiece.y)) {
                    currentPiece.x--;
                    updateGhostY();
                    needsRender = true;
                }
                dasTimer = 0;
            } else {
                dasTimer++;
                if (dasTimer >= DAS_DELAY && (dasTimer - DAS_DELAY) % ARR_RATE === 0) {
                    if (!checkCollision(currentPiece, currentPiece.x - 1, currentPiece.y)) {
                        currentPiece.x--;
                        updateGhostY();
                        needsRender = true;
                    }
                }
            }
        } else if (joyState.right) {
            if (!lastJoyState.right) {
                if (!checkCollision(currentPiece, currentPiece.x + 1, currentPiece.y)) {
                    currentPiece.x++;
                    updateGhostY();
                    needsRender = true;
                }
                dasTimer = 0;
            } else {
                dasTimer++;
                if (dasTimer >= DAS_DELAY && (dasTimer - DAS_DELAY) % ARR_RATE === 0) {
                    if (!checkCollision(currentPiece, currentPiece.x + 1, currentPiece.y)) {
                        currentPiece.x++;
                        updateGhostY();
                        needsRender = true;
                    }
                }
            }
        } else {
            dasTimer = 0;
        }

        if (joyState.up && !lastJoyState.up) {
            const rotatedPiece = { ...currentPiece, shape: rotateMatrix(currentPiece.shape) };
            if (!checkCollision(rotatedPiece, rotatedPiece.x, rotatedPiece.y)) {
                currentPiece.shape = rotatedPiece.shape;
                updateGhostY();
                needsRender = true;
            }
        }

        if (joyState.down) {
            if (!checkCollision(currentPiece, currentPiece.x, currentPiece.y + 1)) {
                currentPiece.y++;
                needsRender = true;
            }
        }

        lastJoyState.left = joyState.left;
        lastJoyState.right = joyState.right;
        lastJoyState.up = joyState.up;
        lastJoyState.down = joyState.down;

        // Gravity
        dropCounter++;
        if (dropCounter >= dropInterval) {
            dropCounter = 0;
            if (!checkCollision(currentPiece, currentPiece.x, currentPiece.y + 1)) {
                currentPiece.y++;
                needsRender = true;
            } else {
                mergePiece();

                // Scoring and Speed up
                const cleared = clearLines();
                if (cleared > 0) {
                    const points = [0, 1, 3, 5, 8]; // 1, 2, 3, or 4 lines
                    score += points[cleared];
                }

                spawnPiece();

                // Game Over check
                if (checkCollision(currentPiece, currentPiece.x, currentPiece.y)) {
                    board = Array.from({ length: ROWS }, () => Array(COLS).fill(null));
                    score = 0;
                    dropInterval = START_SPEED;
                }
                needsRender = true;
            }
        }

        // --- 2. Render Frame (Only if state changed) ---
        if (needsRender) {
            const scene = new Collection({ x: 0, y: 0, color: [0, 0, 0, 255] });

            // Main Board Border
            scene.add(new Rectangle({
                x: OFFSET_X - 1, y: OFFSET_Y - 1,
                width: (COLS * CELL_SIZE) + 2, height: (ROWS * CELL_SIZE) + 2,
                color: [50, 50, 50, 255], fill: false
            }));

            // Static blocks
            for (let y = 0; y < ROWS; y++) {
                for (let x = 0; x < COLS; x++) {
                    if (board[y][x]) {
                        scene.add(new Rectangle({
                            x: OFFSET_X + (x * CELL_SIZE),
                            y: OFFSET_Y + (y * CELL_SIZE),
                            width: CELL_SIZE,
                            height: CELL_SIZE,
                            color: board[y][x],
                            fill: true
                        }));
                    }
                }
            }

            // Current Piece & Ghost
            for (let y = 0; y < currentPiece.shape.length; y++) {
                for (let x = 0; x < currentPiece.shape[y].length; x++) {
                    if (currentPiece.shape[y][x]) {
                        // Ghost
                        scene.add(new Rectangle({
                            x: OFFSET_X + ((currentPiece.x + x) * CELL_SIZE),
                            y: OFFSET_Y + ((ghostY + y) * CELL_SIZE),
                            width: CELL_SIZE,
                            height: CELL_SIZE,
                            color: currentPiece.ghostColor,
                            fill: false,
                            z: 0,
                        }));

                        // Active Piece
                        scene.add(new Rectangle({
                            x: OFFSET_X + ((currentPiece.x + x) * CELL_SIZE),
                            y: OFFSET_Y + ((currentPiece.y + y) * CELL_SIZE),
                            width: CELL_SIZE,
                            height: CELL_SIZE,
                            color: currentPiece.color,
                            fill: true,
                            z: 1,
                        }));
                    }
                }
            }

            // Next Piece Preview
            const previewStartX = OFFSET_X + (COLS * CELL_SIZE) + 4;
            const previewStartY = OFFSET_Y + 5;

            for (let y = 0; y < nextPieceTemplate.shape.length; y++) {
                for (let x = 0; x < nextPieceTemplate.shape[y].length; x++) {
                    if (nextPieceTemplate.shape[y][x]) {
                        scene.add(new Rectangle({
                            x: previewStartX + (x * CELL_SIZE),
                            y: previewStartY + (y * CELL_SIZE),
                            width: CELL_SIZE,
                            height: CELL_SIZE,
                            color: nextPieceTemplate.color,
                            fill: true
                        }));
                    }
                }
            }

            renderer.render(scene, renderBuffer, true, Format.RGB_565_LITTLE, -1);

            renderer.drawText(
                renderBuffer,
                score.toString(),
                2, 2,
                font,
                [255, 255, 255, 255],
                false,
                Format.RGB_565_LITTLE,
                -1
            );

            sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);

            needsRender = false;
        }

        // --- 3. Delay ---
        await sleep(POLL_INTERVAL);
    }
}
