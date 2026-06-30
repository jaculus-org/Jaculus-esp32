import { Renderer } from 'renderer';
import { Collection, Rectangle } from 'shapes';
import { Format } from '../constants.js';
import * as adc from "adc";
import * as gpio from "gpio";
import { buildModesetBuffer, buildSyncBuffer, sendRpHub75Frame, setupSpi } from '../spiSender.js';

// --- CONFIGURATION ---
const PANEL_WIDTH = 64;
const PANEL_HEIGHT = 64;
const GRID_SIZE = 16;
const CELL_SIZE = PANEL_WIDTH / GRID_SIZE;

const ADC_X = 4;
const ADC_Y = 5;
adc.configure(ADC_X);
adc.configure(ADC_Y);

const DEADZONE = 300; // Threshold from center (512) to register movement

function getJoystickDirection(currentDir) {
    const xVal = adc.read(ADC_X);
    const yVal = adc.read(ADC_Y);

    // Calculate displacement from center (512)
    const dx = xVal - 512;
    const dy = yVal - 512;

    // Determine which axis has the stronger input
    if (Math.abs(dx) > DEADZONE || Math.abs(dy) > DEADZONE) {
        if (Math.abs(dx) > Math.abs(dy)) {
            if (dx > 0 && currentDir.x === 0) return { x: 1, y: 0 };  // Right
            if (dx < 0 && currentDir.x === 0) return { x: -1, y: 0 }; // Left
        } else {
            if (dy > 0 && currentDir.y === 0) return { x: 0, y: 1 };  // Down
            if (dy < 0 && currentDir.y === 0) return { x: 0, y: -1 }; // Up
        }
    }
    return currentDir;
}

// --- GAME LOGIC ---
let snake = [{ x: 8, y: 8 }, { x: 7, y: 8 }, { x: 6, y: 8 }];
let direction = { x: 1, y: 0 };
let food = { x: 4, y: 4 };

function newFood() {
    return {
        x: Math.floor(Math.random() * GRID_SIZE),
        y: Math.floor(Math.random() * GRID_SIZE)
    };
}

export async function runSnake(startSpi: boolean) {
    if (startSpi) { setupSpi(); }
    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT);
    const renderBuffer = new ArrayBuffer(PANEL_WIDTH * PANEL_HEIGHT * 2);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_565_LITTLE);

    const FRAME_TIME = 15;
    const POLL_INTERVAL = 1;

    while (gpio.read(7)) {
        // --- 1. Update Logic ---
        let next = { x: snake[0].x + direction.x, y: snake[0].y + direction.y };

        // Wrap around
        next.x = (next.x + GRID_SIZE) % GRID_SIZE;
        next.y = (next.y + GRID_SIZE) % GRID_SIZE;

        // Collision Check (Self)
        if (snake.find(p => p.x === next.x && p.y === next.y)) {
            snake = [{ x: 8, y: 8 }, { x: 7, y: 8 }, { x: 6, y: 8 }];
        } else {
            snake.unshift(next);
            if (next.x === food.x && next.y === food.y) {
                food = newFood();
            } else {
                snake.pop();
            }
        }

        // --- 2. Render Frame ---
        const scene = new Collection({ x: 0, y: 0, color: [0, 0, 0, 255] });

        for (let pos of snake) {
            scene.add(new Rectangle({
                x: pos.x * CELL_SIZE,
                y: pos.y * CELL_SIZE,
                width: CELL_SIZE - 1,
                height: CELL_SIZE - 1,
                color: [0, 255, 0, 255],
                fill: true
            }));
        }

        scene.add(new Rectangle({
            x: food.x * CELL_SIZE,
            y: food.y * CELL_SIZE,
            width: CELL_SIZE,
            height: CELL_SIZE,
            color: [255, 0, 0, 255],
            fill: true
        }));

        renderer.render(scene, renderBuffer, true, Format.RGB_565_LITTLE, -1);
        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);

        // --- 3. Responsive Waiting (Polling Phase) ---
        let elapsed = 0;
        while (elapsed < FRAME_TIME) {
            direction = getJoystickDirection(direction);
            await sleep(POLL_INTERVAL);
            elapsed += POLL_INTERVAL;
        }
    }
}
