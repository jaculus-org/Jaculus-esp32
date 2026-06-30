import { Renderer, Font } from 'renderer';
import { Collection, Color, Rectangle } from 'shapes';
import { Format } from '../constants.js';
import * as adc from "adc";
import * as gpio from "gpio";
import { buildModesetBuffer, buildSyncBuffer, sendRpHub75Frame, setupSpi } from '../spiSender.js';

// --- CONFIGURATION ---
const PANEL_WIDTH = 64;
const PANEL_HEIGHT = 64;
const GRID_SIZE = 8;
const CELL_SIZE = PANEL_WIDTH / GRID_SIZE;

// --- HARDWARE CONFIG ---
const MOVE_X = 4;
const MOVE_Y = 5;
const AIM_X = 16;
const AIM_Y = 15;

adc.configure(MOVE_X);
adc.configure(MOVE_Y);
adc.configure(AIM_X);
adc.configure(AIM_Y);

const CENTER = 512;
const DEADZONE = 200;

// --- COLOR CONSTANTS (Avoid creating arrays every frame) ---
const COLOR_HIDDEN: Color = [50, 50, 50, 255]
const COLOR_REVEALED: Color = [180, 180, 180, 255]
const COLOR_MINE: Color = [255, 0, 0, 255]
const COLOR_FLAG: Color = [255, 255, 0, 255]
const COLOR_CURSOR: Color = [255, 255, 255, 255]
const COLOR_TEXT_NUM: Color = [0, 255, 0, 255]
const COLOR_TEXT_MSG: Color = [0, 255, 0, 255]

// --- INPUT UTILITIES ---
function getStickVector(pinX, pinY) {
    const rawX = adc.read(pinX) - CENTER;
    const rawY = adc.read(pinY) - CENTER;

    if (Math.abs(rawX) < DEADZONE && Math.abs(rawY) < DEADZONE) {
        return { x: 0, y: 0, active: false };
    }

    // Optimization: Avoid Math.sqrt for simple grid movement
    // We only need the relative strength of X vs Y
    return {
        x: rawX,
        y: rawY,
        active: true
    };
}

type Cell = {
    isMine: boolean;
    isRevealed: boolean;
    isFlagged: boolean;
    neighborMines: number;
};

type GameStatus = 'PLAYING' | 'WON' | 'LOST';

// --- PERSISTENT GAME STATE ---
let grid: Cell[][] = [];
let cursor = { x: 0, y: 0 };
let status: GameStatus = 'PLAYING';
let mineCount = 10;
let moveCooldown = 0;
let actionCooldown = 0;

// Persistent Rendering Objects
let scene: Collection;
let cellShapes: Rectangle[][] = [];
let cursorShape: Rectangle;

function initGame() {
    status = 'PLAYING';
    cursor = { x: 0, y: 0 };

    // 1. Initialize Logic Grid
    grid = Array.from({ length: GRID_SIZE }, () =>
        Array.from({ length: GRID_SIZE }, () => ({
            isMine: false,
            isRevealed: false,
            isFlagged: false,
            neighborMines: 0
        }))
    );

    // 2. Place Mines
    let placed = 0;
    while (placed < mineCount) {
        const rx = Math.floor(Math.random() * GRID_SIZE);
        const ry = Math.floor(Math.random() * GRID_SIZE);
        if (!grid[ry][rx].isMine) {
            grid[ry][rx].isMine = true;
            placed++;
        }
    }

    // 3. Calculate Neighbors
    for (let y = 0; y < GRID_SIZE; y++) {
        for (let x = 0; x < GRID_SIZE; x++) {
            if (grid[y][x].isMine) continue;
            let count = 0;
            for (let dy = -1; dy <= 1; dy++) {
                for (let dx = -1; dx <= 1; dx++) {
                    const ny = y + dy, nx = x + dx;
                    if (ny >= 0 && ny < GRID_SIZE && nx >= 0 && nx < GRID_SIZE && grid[ny][nx].isMine) {
                        count++;
                    }
                }
            }
            grid[y][x].neighborMines = count;
        }
    }
}

function reveal(x: number, y: number) {
    if (x < 0 || x >= GRID_SIZE || y < 0 || y >= GRID_SIZE) return;
    const cell = grid[y][x];
    if (cell.isRevealed || cell.isFlagged) return;

    cell.isRevealed = true;
    if (cell.isMine) {
        status = 'LOST';
        return;
    }
    if (cell.neighborMines === 0) {
        for (let dy = -1; dy <= 1; dy++) {
            for (let dx = -1; dx <= 1; dx++) {
                reveal(x + dx, y + dy);
            }
        }
    }
}

function checkWin() {
    for (let y = 0; y < GRID_SIZE; y++) {
        for (let x = 0; x < GRID_SIZE; x++) {
            if (!grid[y][x].isMine && !grid[y][x].isRevealed) return false;
        }
    }
    status = 'WON';
    return true;
}

export async function runMinesweeper(startSpi: boolean) {
    if (startSpi) { setupSpi(); }

    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT);
    const font = new Font();
    const renderBuffer = new ArrayBuffer(PANEL_WIDTH * PANEL_HEIGHT * 2);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_565_LITTLE);

    // --- ONE-TIME SCENE SETUP ---
    scene = new Collection({ x: 0, y: 0, color: [0, 0, 0, 255] });
    cellShapes = [];
    for (let y = 0; y < GRID_SIZE; y++) {
        const row: Rectangle[] = [];
        for (let x = 0; x < GRID_SIZE; x++) {
            const rect = new Rectangle({
                x: x * CELL_SIZE,
                y: y * CELL_SIZE,
                width: CELL_SIZE - 1,
                height: CELL_SIZE - 1,
                color: COLOR_HIDDEN,
                fill: true
            });
            row.push(rect);
            scene.add(rect);
        }
        cellShapes.push(row);
    }
    cursorShape = new Rectangle({
        x: 0, y: 0,
        width: CELL_SIZE + 1,
        height: CELL_SIZE + 1,
        color: COLOR_CURSOR,
        fill: false
    });
    scene.add(cursorShape);

    initGame();

    while (gpio.read(7)) {
        // --- 1. Handle Input ---
        const moveInput = getStickVector(MOVE_X, MOVE_Y);
        const aimInput = getStickVector(AIM_X, AIM_Y);

        if (moveCooldown <= 0 && moveInput.active) {
            if (Math.abs(moveInput.x) > Math.abs(moveInput.y)) {
                cursor.x = Math.max(0, Math.min(GRID_SIZE - 1, cursor.x + (moveInput.x > 0 ? 1 : -1)));
            } else {
                cursor.y = Math.max(0, Math.min(GRID_SIZE - 1, cursor.y + (moveInput.y > 0 ? 1 : -1)));
            }
            moveCooldown = 12;
        }

        if (actionCooldown <= 0 && aimInput.active) {
            if (status === 'PLAYING') {
                if (aimInput.x > 200) { // Simplified threshold
                    reveal(cursor.x, cursor.y);
                    checkWin();
                    actionCooldown = 20;
                } else if (aimInput.x < -200) {
                    grid[cursor.y][cursor.x].isFlagged = !grid[cursor.y][cursor.x].isFlagged;
                    actionCooldown = 20;
                }
            } else if (aimInput.x > 200) {
                initGame();
                actionCooldown = 30;
            }
        }

        if (moveCooldown > 0) moveCooldown--;
        if (actionCooldown > 0) actionCooldown--;

        // --- 2. Update Persistent Shapes (No new objects!) ---
        for (let y = 0; y < GRID_SIZE; y++) {
            for (let x = 0; x < GRID_SIZE; x++) {
                const cell = grid[y][x];
                const shape = cellShapes[y][x];

                if (cell.isRevealed) {
                    shape.setColor(cell.isMine ? COLOR_MINE : COLOR_REVEALED);
                } else if (cell.isFlagged) {
                    shape.setColor(COLOR_FLAG);
                } else {
                    shape.setColor(COLOR_HIDDEN);
                }
            }
        }

        cursorShape.setPosition(cursor.x * CELL_SIZE - 1, cursor.y * CELL_SIZE - 1);

        // Render the existing scene
        renderer.render(scene, renderBuffer, true, Format.RGB_565_LITTLE, -1);

        // --- 3. Render Text Overlay ---
        if (status === 'PLAYING') {
            for (let y = 0; y < GRID_SIZE; y++) {
                for (let x = 0; x < GRID_SIZE; x++) {
                    const cell = grid[y][x];
                    if (cell.isRevealed && cell.neighborMines > 0 && !cell.isMine) {
                        renderer.drawText(
                            renderBuffer,
                            cell.neighborMines.toString(),
                            x * CELL_SIZE + 2,
                            y * CELL_SIZE,
                            font,
                            COLOR_TEXT_NUM,
                            false,
                            Format.RGB_565_LITTLE,
                            -1
                        );
                    }
                }
            }
        } else {
            const msg = status === 'WON' ? "WIN!" : "BOOM!";
            renderer.drawText(renderBuffer, msg, 15, 30, font, COLOR_TEXT_MSG, false, Format.RGB_565_LITTLE, -1);
        }

        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);
        await sleep(10);
    }
}
