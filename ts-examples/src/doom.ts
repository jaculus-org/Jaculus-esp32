import { Renderer } from 'renderer';
import { Collection, Rectangle } from 'shapes';
import { Format } from './constants.js';
import * as adc from "adc";
import { buildModesetBuffer, buildSyncBuffer, sendRpHub75Frame, setupSpi } from './spiSender.js';

// --- CONFIGURATION & SPI SETUP ---
const PANEL_WIDTH = 64;
const PANEL_HEIGHT = 64;

// Left Joystick for Movement (Y = Forward/Back, X = Turn Left/Right)
const JOY_X = 4;
const JOY_Y = 5;

adc.configure(JOY_X);
adc.configure(JOY_Y);
const CENTER = 512;
const DEADZONE = 200;

// --- THE 2D MAP ---
// 1 = Blue Wall, 2 = Red Wall, 3 = Green Wall, 0 = Empty Space
const worldMap = [
    [1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 2, 1],
    [1, 0, 3, 0, 3, 0, 2, 1],
    [1, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 3, 0, 3, 0, 2, 1],
    [1, 0, 0, 0, 0, 0, 0, 1],
    [1, 2, 2, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1]
];

// --- CAMERA & PLAYER STATE ---
let posX = 1.5; // Player start X
let posY = 1.5; // Player start Y
let dirX = 1.0; // Initial direction vector
let dirY = 0.0;
let planeX = 0.0; // 2D Raycaster camera plane
let planeY = 0.66; // FOV adjustment

const moveSpeed = 0.15;
const rotSpeed = 0.1;

async function runRaycaster() {
    setupSpi();
    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT);
    const renderBuffer = new ArrayBuffer(PANEL_WIDTH * PANEL_HEIGHT * 2);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_565_LITTLE);

    // We reuse this scene object every frame to avoid massive memory garbage collection
    const scene = new Collection({ x: 0, y: 0, color: [0, 0, 0, 255] });

    while (true) {
        // --- 1. INPUT & MOVEMENT ---
        let rawX = adc.read(JOY_X) - CENTER;
        let rawY = adc.read(JOY_Y) - CENTER;

        if (Math.abs(rawY) > DEADZONE) { // Forward / Backward
            let moveDir = rawY < 0 ? 1 : -1;

            if (worldMap[Math.floor(posX + dirX * moveSpeed * moveDir)][Math.floor(posY)] === 0) {
                posX += dirX * moveSpeed * moveDir;
            }
            if (worldMap[Math.floor(posX)][Math.floor(posY + dirY * moveSpeed * moveDir)] === 0) {
                posY += dirY * moveSpeed * moveDir;
            }
        }

        if (Math.abs(rawX) > DEADZONE) { // Turn Left / Right
            let turnDir = rawX > 0 ? 1 : -1;
            let rs = rotSpeed * turnDir;

            let oldDirX = dirX;
            dirX = dirX * Math.cos(-rs) - dirY * Math.sin(-rs);
            dirY = oldDirX * Math.sin(-rs) + dirY * Math.cos(-rs);

            let oldPlaneX = planeX;
            planeX = planeX * Math.cos(-rs) - planeY * Math.sin(-rs);
            planeY = oldPlaneX * Math.sin(-rs) + planeY * Math.cos(-rs);
        }

        // --- 2. RAYCASTING ENGINE ---
        scene.clear();

        for (let x = 0; x < PANEL_WIDTH; x++) {
            let cameraX = 2 * x / PANEL_WIDTH - 1;
            let rayDirX = dirX + planeX * cameraX;
            let rayDirY = dirY + planeY * cameraX;

            let mapX = Math.floor(posX);
            let mapY = Math.floor(posY);

            let sideDistX, sideDistY;
            let deltaDistX = Math.abs(1 / rayDirX);
            let deltaDistY = Math.abs(1 / rayDirY);
            let perpWallDist;

            let stepX, stepY;
            let hit = 0;
            let side;

            if (rayDirX < 0) {
                stepX = -1;
                sideDistX = (posX - mapX) * deltaDistX;
            } else {
                stepX = 1;
                sideDistX = (mapX + 1.0 - posX) * deltaDistX;
            }
            if (rayDirY < 0) {
                stepY = -1;
                sideDistY = (posY - mapY) * deltaDistY;
            } else {
                stepY = 1;
                sideDistY = (mapY + 1.0 - posY) * deltaDistY;
            }

            while (hit === 0) {
                if (sideDistX < sideDistY) {
                    sideDistX += deltaDistX;
                    mapX += stepX;
                    side = 0;
                } else {
                    sideDistY += deltaDistY;
                    mapY += stepY;
                    side = 1;
                }
                if (worldMap[mapX][mapY] > 0) hit = 1;
            }

            if (side === 0) {
                perpWallDist = (mapX - posX + (1 - stepX) / 2) / rayDirX;
            } else {
                perpWallDist = (mapY - posY + (1 - stepY) / 2) / rayDirY;
            }

            let lineHeight = Math.floor(PANEL_HEIGHT / perpWallDist);

            let drawStart = -lineHeight / 2 + PANEL_HEIGHT / 2;
            if (drawStart < 0) drawStart = 0;
            let drawEnd = lineHeight / 2 + PANEL_HEIGHT / 2;
            if (drawEnd >= PANEL_HEIGHT) drawEnd = PANEL_HEIGHT - 1;

            let wallColor;
            let tile = worldMap[mapX][mapY];
            if (tile === 1) wallColor = [0, 0, 200, 255];       // Blue
            else if (tile === 2) wallColor = [200, 0, 0, 255];  // Red
            else if (tile === 3) wallColor = [0, 200, 0, 255];  // Green
            else wallColor = [200, 200, 200, 255];

            if (side === 1) {
                wallColor = [wallColor[0] / 2, wallColor[1] / 2, wallColor[2] / 2, 255];
            }

            scene.add(new Rectangle({
                x: PANEL_WIDTH - drawEnd,
                y: x,
                width: drawEnd - drawStart,
                height: 1,
                color: wallColor,
                fill: true
            }));
        }

        // --- 3. RENDER FRAME ---
        renderer.render(scene, renderBuffer, false, Format.RGB_565_LITTLE);
        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);

        await sleep(1);
    }
}

runRaycaster();
