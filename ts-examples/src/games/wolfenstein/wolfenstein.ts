import * as adc from "adc";
import * as gpio from "gpio";
import { Raycaster } from 'raycaster';
import { Format } from '../../constants.js';
import {
    buildModesetBuffer,
    buildSyncBuffer,
    sendRpHub75Frame,
    setupSpi
} from '../../spiSender.js';
import { BLUESTONE_TEX, DOOR_TEX, GUARD_SHOOT1_TEX, GUARD_SHOOT2_TEX, GUARD_TEX, GUARD_WALK1_TEX, GUARD_WALK2_TEX, GUARD_WALK3_TEX, GUARD_WALK4_TEX, GUN_FIRE_TEX, GUN_TEX, HEALTH_TEX, PALETTE, STONE_TEX, texH, texW, WOOD_TEX } from "./sprites.js";
import { Door, EnemyEntity, TextureType } from './types.js';
import { BULLET_SPEED, BULLET_TEX, CENTER, DEADZONE, DOOR_EW_TILES, DOOR_NS_TILES, entities, INITIAL_X, INITIAL_Y, JOY_MOVE_X, JOY_MOVE_Y, JOY_X, JOY_Y, PALETTE_original, PANEL_HEIGHT, PANEL_WIDTH, WALL_TILES, worldMap } from "./config.js";
import { updateEnemyAI } from "./ai.js";

async function bakeTexture(grid: string, palette: Record<string, number>): Promise<ArrayBuffer> {
    const buffer = new Uint16Array(grid.length);
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

        if (bufIdx % 2048 === 0) {
            await sleep(1);
        }
    }

    return buffer.slice(0, bufIdx).buffer;
}

let activeDoors: Door[] = [];

// --- GAME STATE ---
export let playerHealth = 100;
let weaponFrame = 0;
let weaponTimer = 0;
let triggerPulled = false;

// --- CAMERA & PLAYER STATE ---
let posX = INITIAL_X;
let posY = INITIAL_Y;
let dirX = 1.0;
let dirY = 0.0;
let planeX = 0.0;
let planeY = 0.66; // FOV adjustment

const moveSpeed = 0.07;
const rotSpeed = 0.07;

export function updateHealth(val: number) {
    playerHealth = val;
}

function rgbTo565(r: number, g: number, b: number): number {
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

function drawHealthBarRaw(buffer: ArrayBuffer, health: number) {
    const view = new Uint16Array(buffer);
    const width = 64;

    const startY = 61;
    const startX = 2;
    const barLength = 30;
    const barThickness = 2;

    const colorBG = rgbTo565(80, 0, 0);      // Dark Red
    let colorFG = rgbTo565(0, 255, 0);       // Green

    const healthPercent = Math.max(0, Math.min(1, health / 100));
    if (healthPercent < 0.3) colorFG = rgbTo565(255, 0, 0);
    else if (healthPercent < 0.6) colorFG = rgbTo565(255, 255, 0);

    const currentFillHeight = Math.floor(barLength * healthPercent);

    for (let y = startY; y > startY - barLength; y--) {
        if (y < 0) break;

        for (let x = startX; x < startX + barThickness; x++) {
            if (x >= width) break;

            const index = (y * width) + x;

            const distanceFromBottom = startY - y;

            if (distanceFromBottom < currentFillHeight) {
                view[index] = colorFG;
            } else {
                view[index] = colorBG;
            }
        }
    }
}
export function tryMove(newX: number, newY: number): boolean {
    let mapX = Math.floor(newX);
    let mapY = Math.floor(newY);
    let tile = worldMap[mapX][mapY];

    if (DOOR_NS_TILES.includes(tile) || DOOR_EW_TILES.includes(tile)) {
        let door = activeDoors.find(d => d.x === mapX && d.y === mapY);
        if (!door) {
            activeDoors.push({ x: mapX, y: mapY, openAmount: 0, state: 'opening', timer: 0 });
            return false;
        } else if (door.openAmount > 0.7) {
            return true;
        }
        return false;
    }

    return !WALL_TILES.includes(tile);
}

function processDoors(): number[] {
    let flatDoorArray: number[] = [];
    let playerMapX = Math.floor(posX);
    let playerMapY = Math.floor(posY);

    for (let i = activeDoors.length - 1; i >= 0; i--) {
        let door = activeDoors[i];

        if (door.state === 'opening') {
            door.openAmount += 0.05;
            if (door.openAmount >= 1.0) {
                door.openAmount = 1.0;
                door.state = 'open';
                door.timer = 60;
            }
        } else if (door.state === 'open') {
            door.timer--;
            let playerInDoor = (playerMapX === door.x && playerMapY === door.y);

            if (door.timer <= 0 && !playerInDoor) {
                door.state = 'closing';
            } else if (playerInDoor && door.timer <= 0) {
                door.timer = 10;
            }
        } else if (door.state === 'closing') {
            door.openAmount -= 0.05;
            if (door.openAmount <= 0.0) {
                activeDoors.splice(i, 1);
                continue;
            }
        }

        flatDoorArray.push(door.x, door.y, door.openAmount);
    }
    return flatDoorArray;
}

function handleMovement(rawMoveX: number, rawMoveY: number, rawX: number) {
    // Forward / Backward
    if (Math.abs(rawMoveX) > DEADZONE) {
        let moveDirX = rawMoveX < 0 ? -1 : 1;
        if (tryMove(posX + dirX * moveSpeed * moveDirX, posY)) posX += dirX * moveSpeed * moveDirX;
        if (tryMove(posX, posY + dirY * moveSpeed * moveDirX)) posY += dirY * moveSpeed * moveDirX;
    }
    // Left / Right (Strafe)
    if (Math.abs(rawMoveY) > DEADZONE) {
        let moveDir = rawMoveY < 0 ? -1 : 1;
        const strafeX = -dirY; const strafeY = dirX;
        if (tryMove(posX + strafeX * moveSpeed * moveDir, posY)) posX += strafeX * moveSpeed * moveDir;
        if (tryMove(posX, posY + strafeY * moveSpeed * moveDir)) posY += strafeY * moveSpeed * moveDir;
    }
    // Turn Left / Right
    if (Math.abs(rawX) > DEADZONE) {
        let turnDir = rawX > 0 ? 1 : -1;
        let rs = rotSpeed * turnDir;
        let oldDirX = dirX; dirX = dirX * Math.cos(-rs) - dirY * Math.sin(-rs); dirY = oldDirX * Math.sin(-rs) + dirY * Math.cos(-rs);
        let oldPlaneX = planeX; planeX = planeX * Math.cos(-rs) - planeY * Math.sin(-rs); planeY = oldPlaneX * Math.sin(-rs) + planeY * Math.cos(-rs);
    }
}


export function spawnBullet(x: number, y: number, dirX: number, dirY: number, owner: 'player' | 'enemy') {
    entities.push({
        x: x,
        y: y,
        tex: 3,
        active: true,
        type: 'bullet',
        scale: 0.2,
        vx: dirX * BULLET_SPEED,
        vy: dirY * BULLET_SPEED,
        owner: owner
    });
}

function handleShooting(rawY: number) {
    if (Math.abs(rawY) > DEADZONE && weaponTimer == 0) {
        triggerPulled = true;
    }

    if (triggerPulled && weaponTimer === 0) {
        weaponTimer = 10;
        triggerPulled = false;
        spawnBullet(posX, posY, dirX, dirY, 'player');
    }

    if (weaponTimer > 0) {
        weaponTimer--;
        if (weaponTimer > 6) weaponFrame = 1;
        else weaponFrame = 2;
    } else {
        weaponFrame = 0;
    }
}

function processEntities(): number[] {
    let flatSpriteArray: number[] = [];
    const CULL_DIST_SQ = 15 * 15;

    for (let ent of entities) {
        if (!ent.active) continue;


        if (ent.type === 'bullet') {
            ent.x += ent.vx!;
            ent.y += ent.vy!;

            let mapX = Math.floor(ent.x);
            let mapY = Math.floor(ent.y);
            if (worldMap[mapX] && worldMap[mapX][mapY] > 0) {
                ent.active = false;
                continue;
            }

            if (ent.owner === 'player') {
                for (let j = 0; j < entities.length; j++) {
                    let other = entities[j];
                    if (other.type === 'enemy' && other.active) {
                        let hitDist = (ent.x - other.x) ** 2 + (ent.y - other.y) ** 2;
                        if (hitDist < 0.16) {
                            const enemy = other as EnemyEntity;
                            enemy.health -= 50;
                            if (enemy.health <= 0) enemy.active = false;
                            ent.active = false;
                            console.log('Enemy hit' + (!enemy.active ? ' KILLED' : ''));
                            break;
                        }
                    }
                }
            } else if (ent.owner === 'enemy') {
                let distToPlayer = (ent.x - posX) ** 2 + (ent.y - posY) ** 2;
                if (distToPlayer < 0.16) {
                    playerHealth -= 10;
                    ent.active = false;
                    console.log(`Player hit by enemy! Health: ${playerHealth}`);
                }
            }
        } else if (ent.type === 'enemy') {
            let distSq = (ent.x - posX) ** 2 + (ent.y - posY) ** 2;
            if (distSq < 25 * 25) {
                updateEnemyAI(ent as EnemyEntity, posX, posY);
            }
        }
        if (!ent.active) continue;

        let distSq = (ent.x - posX) ** 2 + (ent.y - posY) ** 2;
        if (distSq > CULL_DIST_SQ) continue;

        const dx = ent.x - posX;
        const dy = ent.y - posY;
        const dot = dx * dirX + dy * dirY;
        if (dot < -0.5) continue;

        if (ent.type === 'potion' && distSq < 0.25) {
            ent.active = false;
            playerHealth = Math.min(playerHealth + 25, 100);
            continue;
        }

        flatSpriteArray.push(ent.x, ent.y, ent.tex, ent.scale);
    }

    return flatSpriteArray;
}

async function addWallTextures(raycaster: Raycaster) {
    raycaster.setTexture(1, await bakeTexture(STONE_TEX, PALETTE), texW, texH, TextureType.Wall);
    raycaster.setTexture(2, await bakeTexture(BLUESTONE_TEX, PALETTE), texW, texH, TextureType.Wall);
    raycaster.setTexture(3, await bakeTexture(WOOD_TEX, PALETTE), texW, texH, TextureType.Wall);

    const bakedDoor = await bakeTexture(DOOR_TEX, PALETTE);
    raycaster.setTexture(4, bakedDoor, texW, texH, TextureType.Wall);
    raycaster.setTexture(5, bakedDoor, texW, texH, TextureType.Wall);

}

async function addGuardTextures(raycaster: Raycaster) {
    let id = 4;
    for (const tex of [GUARD_WALK1_TEX, GUARD_WALK2_TEX, GUARD_WALK3_TEX, GUARD_WALK4_TEX, GUARD_SHOOT1_TEX, GUARD_SHOOT2_TEX]) {
        raycaster.setTexture(id, await bakeTexture(tex, PALETTE), texW, texH, TextureType.Sprite);
        id++;
    }
}

async function addSpriteTextures(raycaster: Raycaster) {
    raycaster.setTexture(1, await bakeTexture(HEALTH_TEX, PALETTE), texW, texH, TextureType.Sprite);
    raycaster.setTexture(2, await bakeTexture(GUARD_TEX, PALETTE), texW, texH, TextureType.Sprite);
    raycaster.setTexture(3, await bakeTexture(BULLET_TEX, PALETTE_original), 16, 16, TextureType.Sprite);

    await addGuardTextures(raycaster);
}

async function addTextures(raycaster: Raycaster) {
    await addWallTextures(raycaster);
    await addSpriteTextures(raycaster);
}


// ==========================================
// MAIN LOOP
// ==========================================
export async function runWolfenstein(startSpi: boolean) {
    if (startSpi) { setupSpi(); }

    const raycaster = new Raycaster(PANEL_WIDTH, PANEL_HEIGHT);
    raycaster.setMap(worldMap);
    raycaster.setTileConfig(WALL_TILES, DOOR_NS_TILES, DOOR_EW_TILES);

    await addTextures(raycaster);


    const floorMap: number[][] = [];
    const ceilingMap: number[][] = [];
    for (let x = 0; x < worldMap.length; x++) {
        floorMap[x] = [];
        ceilingMap[x] = [];
        for (let y = 0; y < worldMap[x].length; y++) {
            if (x > 5 && x < 10 && y > 5 && y < 10) {
                floorMap[x][y] = PALETTE['R1'];
                ceilingMap[x][y] = PALETTE['D1'];
            } else {
                floorMap[x][y] = 0x2104;
                ceilingMap[x][y] = 0x0000;
            }
        }
    }

    raycaster.setFloorMap(floorMap);
    raycaster.setCeilingMap(ceilingMap);

    const bakedGun = await bakeTexture(GUN_TEX, PALETTE);
    raycaster.setTexture(0, bakedGun, texW, texH, TextureType.Weapon);
    raycaster.setTexture(1, await bakeTexture(GUN_FIRE_TEX, PALETTE), texW, texH, TextureType.Weapon);
    raycaster.setTexture(2, bakedGun, texW, texH, TextureType.Weapon);

    const renderBuffer = new ArrayBuffer(PANEL_WIDTH * PANEL_HEIGHT * 2);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_565_LITTLE);

    while (gpio.read(7)) {
        // --- 1. INPUT ---
        let rawX = adc.read(JOY_X) - CENTER;
        let rawY = adc.read(JOY_Y) - CENTER;
        let rawMoveX = adc.read(JOY_MOVE_X) - CENTER;
        let rawMoveY = adc.read(JOY_MOVE_Y) - CENTER;

        // --- 2. LOGIC ---
        handleMovement(rawMoveX, rawMoveY, rawX);
        handleShooting(rawY);
        let flatSpriteArray = processEntities();
        let flatDoorArray = processDoors();

        // --- 3. RENDER ---
        raycaster.render(
            renderBuffer,
            posX, posY, dirX, dirY, planeX, planeY,
            flatSpriteArray,
            flatDoorArray,
            weaponFrame,
            Format.RGB_565_LITTLE
        );

        drawHealthBarRaw(renderBuffer, playerHealth);

        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);

        await sleep(1);
    }
}
