import { runPong } from "./pong.js";
import { runDoom } from "./doom.js";
import { runTetris } from "./tetris.js";
import { runAsteroids } from "./asteroids.js";
import { runSnake } from "./snakeDisplay.js";
import { runWolfenstein } from "./wolfenstein/wolfenstein.js";
import { buildModesetBuffer, buildSyncBuffer, sendRpHub75Frame, setupSpi } from "../spiSender.js";
import { PANEL_HEIGHT, PANEL_WIDTH } from "./wolfenstein/config.js";
import { Format } from "../constants.js";
import * as adc from "adc";
import * as gpio from "gpio";
import { runMinesweeper } from "./minesweeper.js";

// --- Hardware Constants ---
const ADC_X = 4;
const ADC_Y = 5;
const DEADZONE = 300;

const SOURCE_ICON_SIZE = 16;
const DISPLAY_ICON_SIZE = Math.floor(PANEL_HEIGHT / 2);
const SCALE = DISPLAY_ICON_SIZE / SOURCE_ICON_SIZE;
const SPACING = DISPLAY_ICON_SIZE + 4;

const INPUT_DELAY = 250;
let lastInputTime = 0;

adc.configure(ADC_X);
adc.configure(ADC_Y);

// --- Display Setup ---
const renderBuffer = new ArrayBuffer(PANEL_WIDTH * PANEL_HEIGHT * 2);
const view = new Uint16Array(renderBuffer);
const syncBuffer = buildSyncBuffer();
const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_565_LITTLE);

// --- Color Helpers ---
function rgb565(r: number, g: number, b: number): number {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


const PALETTE: Record<string, number> = {
    '.': 0x0000,                  // Transparent
    'W': rgb565(255, 255, 255),    // White
    'w': rgb565(180, 180, 180),    // Light Gray
    'S': rgb565(100, 100, 100),    // Dark Gray
    'R': rgb565(255, 0, 0),        // Red
    'r': rgb565(150, 0, 0),        // Dark Red
    'G': rgb565(0, 255, 0),        // Green
    'g': rgb565(0, 120, 0),        // Dark Green
    'B': rgb565(50, 50, 255),      // Blue
    'b': rgb565(0, 0, 180),        // Dark Blue
    'Y': rgb565(255, 255, 0),      // Yellow
    'O': rgb565(255, 120, 0),      // Orange
};

// --- Icon Definitions (ASCII Art) ---
// Each icon must be 8x8 characters
// --- Improved Icon Definitions (16x16) ---
const ICONS = {
    // A dynamic shot with a ball trail and two paddles
    pong: `
................
................
.WW.............
.WW.............
.WW......w......
.WW.....W.......
.WW.............
................
................
............WW..
.......W....WW..
............WW..
............WW..
............WW..
................
................`,

    // Stylized Cacodemon (Red demon with one big eye)
    doom: `
.....rrrrrr.....
...rrRRRRRRrr...
..rRRRRRRRRRRr..
.rRRRRRRRRRRRRr.
.rRRWWWWWWWWWRr.
.rRRWWBWWWBWWRr.
.rRRWWBBBBBWWrr.
.rRRWWWWWWWWWRr.
.rRRRRRRRRRRRRr.
.rRRRRRRRRRRRRr.
..rRRRRRRRRRRr..
..rRROOOOOORRr..
...rRROOOORRr...
....rrrrrrrr....
................
................`,

    // Classic Tetrominoes (Z, O, and I pieces)
    tetris: `
................
......BBBB......
......BBBB......
................
...RRRR.........
...RRRR.........
......RRRR......
......RRRR......
................
..........OO....
...YYYY...OO....
...YYYY...OO....
...YYYY...OOOO..
...YYYY...OOOO..
................
................`,

    // Triangular ship firing at a craggy asteroid
    asteroids: `
................
....wwwwww......
..wwSSSSSSww....
.wSSSSSSSSSSw...
.wSSSSSSSSSSw...
..wwSSSSSSww....
....wwwwww......
.......WW.......
.......WW.......
................
.......W........
......WWW.......
.....W.W.W......
....W..W..W.....
................
................`,
    // A snake with a distinct head (eyes) approaching a red apple
    snake: `
................
..GGGGGGGG......
..GggggggG......
..Gg....gG......
..Gg..R.gG......
..Gg....gG......
..Gg.GGGGg......
..Gg.GgggG......
..Gg.Gg.........
..Gg.Gg.........
..Gg.GGGGGGGG...
..Gg.GggggggG...
..Gg........G...
..GGGGGGGGGGG...
................
................`,

    // A 3D perspective hallway (Blue walls, gray floor/ceiling)
    wolf: `
bbbbbbbbbbbbbbbb
bWWWWWWWWWWWWWWb
bWSSSSSSSSSSSSWb
bWSssssssssssSWb
bWSsWWWWWWWWsSWb
bWSsW......WsSWb
bWSsW..ss..WsSWb
bWSsW..ss..WsSWb
bWSsW..SS..WsSWb
bWSsW..SS..WsSWb
bWSsWWWWWWWWsSWb
bWSssssssssssSWb
bWSSSSSSSSSSSSWb
bWWWWWWWWWWWWWWb
bbbbbbbbbbbbbbbb
................`
};
interface GameEntry {
    name: string;
    run: (startSpi: boolean) => void;
    iconStr: string;
    bakedIcon: Uint16Array | null;
}

const GAMES: GameEntry[] = [
    { name: "Pong", run: runPong, iconStr: ICONS.pong, bakedIcon: null },
    { name: "Minesweeper", run: runMinesweeper, iconStr: ICONS.asteroids, bakedIcon: null },
    { name: "Doom", run: runDoom, iconStr: ICONS.doom, bakedIcon: null },
    { name: "Tetris", run: runTetris, iconStr: ICONS.tetris, bakedIcon: null },
    { name: "Asteroids", run: runAsteroids, iconStr: ICONS.asteroids, bakedIcon: null },
    { name: "Snake", run: runSnake, iconStr: ICONS.snake, bakedIcon: null },
    { name: "Wolfenstein3D", run: runWolfenstein, iconStr: ICONS.wolf, bakedIcon: null },
];

// --- Texture Engine ---

async function bakeTexture(grid: string, palette: Record<string, number>): Promise<ArrayBuffer> {
    const buffer = new Uint16Array(grid.length * 2); // Over-allocate
    let bufIdx = 0;
    let i = 0;

    while (i < grid.length) {
        const char = grid[i];
        if (char === ' ' || char === '\n' || char === '\r' || char === '\t') {
            i++;
            continue;
        }

        let matched = false;
        for (let len = 2; len > 0; len--) {
            if (i + len > grid.length) continue;
            const token = grid.substring(i, i + len);
            if (palette[token] !== undefined) {
                buffer[bufIdx++] = palette[token];
                i += len;
                matched = true;
                break;
            }
        }

        if (!matched) {
            buffer[bufIdx++] = 0x0000;
            i++;
        }

        if (bufIdx % 2048 === 0) await sleep(1);
    }
    return buffer.slice(0, bufIdx).buffer;
}

function blitScaledIcon(texture: Uint16Array, startX: number, startY: number) {
    for (let row = 0; row < SOURCE_ICON_SIZE; row++) {
        for (let col = 0; col < SOURCE_ICON_SIZE; col++) {
            const texPixel = texture[row * SOURCE_ICON_SIZE + col];
            if (texPixel === 0x0000) continue;

            for (let dy = 0; dy < SCALE; dy++) {
                for (let dx = 0; dx < SCALE; dx++) {
                    const logicalX = Math.floor(startX + (col * SCALE) + dx);
                    const logicalY = Math.floor(startY + (row * SCALE) + dy);

                    const screenX = logicalY;
                    const screenY = (PANEL_HEIGHT - 1) - logicalX;

                    if (screenX >= 0 && screenX < PANEL_WIDTH && screenY >= 0 && screenY < PANEL_HEIGHT) {
                        view[screenY * PANEL_WIDTH + screenX] = texPixel;
                    }
                }
            }
        }
    }
}
// --- Input Logic ---

function getJoystickDirection(currentDir: { x: number, y: number }) {
    const xVal = adc.read(ADC_X);
    const yVal = adc.read(ADC_Y);
    const dx = xVal - 512;
    const dy = yVal - 512;

    if (Math.abs(dx) > DEADZONE || Math.abs(dy) > DEADZONE) {
        if (Math.abs(dx) > Math.abs(dy)) {
            if (dx > 0 && currentDir.x === 0) return { x: 1, y: 0 };
            if (dx < 0 && currentDir.x === 0) return { x: -1, y: 0 };
        } else {
            if (dy > 0 && currentDir.y === 0) return { x: 0, y: 1 };
            if (dy < 0 && currentDir.y === 0) return { x: 0, y: -1 };
        }
    }
    return currentDir;
}

// --- Main Process ---
function drawRotatedRect(x: number, y: number, w: number, h: number, color: number) {
    for (let i = 0; i < w; i++) {
        for (let j = 0; j < h; j++) {
            if (i === 0 || i === w - 1 || j === 0 || j === h - 1) {
                const sX = Math.floor(y + j);
                const sY = Math.floor((PANEL_HEIGHT - 1) - (x + i));
                if (sX >= 0 && sX < PANEL_WIDTH && sY >= 0 && sY < PANEL_HEIGHT) {
                    view[sY * PANEL_WIDTH + sX] = color;
                }
            }
        }
    }
}

async function initAndRun() {
    setupSpi();
    gpio.pinMode(7, gpio.PinMode.INPUT_PULLUP);
    for (const game of GAMES) {
        const buffer = await bakeTexture(game.iconStr, PALETTE);
        game.bakedIcon = new Uint16Array(buffer);
    }

    let selectedIndex = 0;
    let scrollY = 0;

    while (true) {
        const now = Date.now();
        const rawDir = getJoystickDirection({ x: 0, y: 0 });

        if (now - lastInputTime > INPUT_DELAY) {
            if (rawDir.y !== 0) {
                if (rawDir.y === -1) {
                    selectedIndex = (selectedIndex - 1 + GAMES.length) % GAMES.length;
                } else if (rawDir.y === 1) {
                    selectedIndex = (selectedIndex + 1) % GAMES.length;
                }
                lastInputTime = now;
            }
        }

        if (rawDir.x === 1) {
            const game = GAMES[selectedIndex];
            await game.run(false);
        }

        const targetScrollY = (PANEL_HEIGHT / 2) - (DISPLAY_ICON_SIZE / 2) - (selectedIndex * SPACING);

        // "Lerp" formula: Current + (Target - Current) * EaseFactor
        // 0.1 is slow and lazy, 0.5 is snappy.
        scrollY = scrollY + (targetScrollY - scrollY) * 0.15;

        // 3. Render Frame
        view.fill(0x0000);

        GAMES.forEach((game, index) => {
            const yPos = scrollY + (index * SPACING);
            const xPos = (PANEL_WIDTH - DISPLAY_ICON_SIZE) / 2;

            if (yPos + DISPLAY_ICON_SIZE < -20 || yPos > PANEL_HEIGHT + 20) return;

            const isSelected = index === selectedIndex;

            // Draw selection highlight with rotation
            if (isSelected) {
                drawRotatedRect(xPos - 2, yPos - 2, DISPLAY_ICON_SIZE + 4, DISPLAY_ICON_SIZE + 4, 0xFFFF);
            }

            if (game.bakedIcon) {
                blitScaledIcon(game.bakedIcon, xPos, yPos);
            }
        });

        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);

        // High frame rate for smooth scrolling (approx 60fps)
        await sleep(16);
    }
}

initAndRun();
