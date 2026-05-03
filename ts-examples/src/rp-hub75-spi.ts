import { Renderer } from 'renderer';
import { Collection, Circle, Rectangle, LineSegment } from 'shapes';
import { SPI2 } from 'spi';
import { Format } from './constants.js';

const PANEL_WIDTH = 64;
const PANEL_HEIGHT = 64;
const MAX_PIXELS = PANEL_WIDTH * PANEL_HEIGHT;
const BUFFER_SIZE_BYTES = MAX_PIXELS * 3;

const PIN_SCK = 3;
const PIN_CS = 1;
const SPI_BAUD = 2_000_000;
const SPI_MODE = 0;
const MODE_MAGIC = 0xfb;

const SYNC_WORDS = [
    0xac92, 0x3bca, 0x41bf, 0x393d, 0xa74a, 0xae01, 0x155d, 0xfb70, 0xf681, 0x2f6d, 0x4931, 0x0fa3, 0x77bf, 0xd756, 0x26f9, 0x4eb6,
];

function setupSpi() {
    SPI2.setup({
        sck: PIN_SCK,
        data0: 38,
        data1: 39,
        data2: 41,
        data3: 45,
        baud: SPI_BAUD,
        mode: SPI_MODE,
        order: 'lsb',
    });
}

function buildSyncBuffer() {
    const buffer = new Uint8Array(SYNC_WORDS.length * 2);
    const view = new DataView(buffer.buffer);

    for (let i = 0; i < SYNC_WORDS.length; i++) {
        view.setUint16(i * 2, SYNC_WORDS[i], true);
    }

    return buffer;
}

function buildModesetBuffer() {
    const buffer = new Uint8Array(8);
    const view = new DataView(buffer.buffer);

    view.setUint8(0, MODE_MAGIC);
    view.setUint8(1, 0);
    view.setUint8(2, Format.RGB_888);
    view.setUint8(3, 255);
    view.setUint16(4, PANEL_WIDTH, true);

    return buffer;
}

function sendRpHub75Frame(syncBuffer: Uint8Array, modesetBuffer: Uint8Array, frameBuffer: ArrayBuffer) {
    SPI2.transfer(syncBuffer, PIN_CS, 0, true);
    SPI2.transfer(modesetBuffer, PIN_CS, 0, true);

    SPI2.transfer(frameBuffer, PIN_CS, 0, true);
}

export async function solarSystemExample() {
    setupSpi();

    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT);
    const renderBuffer = new ArrayBuffer(BUFFER_SIZE_BYTES);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer();

    const sunCollection = new Collection({ x: 0, y: 0, color: [0, 0, 0, 255], z: 0 });
    sunCollection.setPivot(32, 32);

    const sun = new Circle({
        x: 32, y: 32,
        radius: 8,
        color: [255, 255, 0, 255],
        fill: true
    });
    sunCollection.add(sun);

    const earthCollection = new Collection({ x: 0, y: 0, color: [0, 0, 0, 255], z: 5 });
    earthCollection.setPivot(32, 32);

    const earth = new Circle({
        x: 32 + 20, y: 32,
        radius: 4,
        color: [0, 150, 255, 255],
        fill: true,
        z: 10
    });
    earthCollection.add(earth);

    const alien = new Circle({
        x: 32 - 20, y: 32,
        radius: 4,
        color: [0, 255, 100, 255],
        fill: true,
        z: 10
    });
    earthCollection.add(alien);

    const moonCollection = new Collection({ x: 0, y: 0, color: [0, 0, 0, 255], z: 10 });
    moonCollection.setPivot(52, 32);

    const moon = new Circle({
        x: 32 + 20 + 8, y: 32,
        radius: 2,
        color: [200, 200, 200, 255],
        fill: true,
        z: 10
    });
    moonCollection.add(moon);

    earthCollection.add(moonCollection);
    sunCollection.add(earthCollection);

    const earthOrbit = new Circle({
        x: 32, y: 32,
        radius: 20,
        color: [100, 100, 100, 255],
        fill: false,
        z: 1
    });
    sunCollection.add(earthOrbit);

    const moonOrbit = new Circle({
        x: 32 + 20, y: 32,
        radius: 8,
        color: [100, 100, 100, 255],
        fill: false,
        z: 1
    });
    earthCollection.add(moonOrbit);

    console.log(`Starting RP-HUB75 SPI loop (${syncBuffer.length} sync bytes + ${modesetBuffer.length} modeset bytes + rendered RGB565 frames)...`);

    while (true) {
        earthCollection.rotate(1.5);
        moonCollection.rotate(3);
        renderer.render(sunCollection, renderBuffer, true, Format.RGB_888);
        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);
        await sleep(1);
    }
}

export async function rectangleExample() {
    setupSpi();

    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT);
    const renderBuffer = new ArrayBuffer(BUFFER_SIZE_BYTES);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer();

    const scene = new Collection({ x: 0, y: 0, color: [0, 0, 0, 255] });
    scene.setPivot(32, 31);
    scene.rotate(-90);
    const rect = new Rectangle({ x: 0, y: 0, width: 5, height: 2, color: [0, 255, 0, 255] });
    scene.add(rect);

    console.log(`Starting RP-HUB75 SPI loop (${syncBuffer.length} sync bytes + ${modesetBuffer.length} modeset bytes + rendered RGB565 frames)...`);

    while (true) {
        rect.setPosition((rect.getX() + 1) % 60, 0);
        renderer.render(scene, renderBuffer, true, Format.RGB_565_LITTLE);
        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);

        await sleep(1);
    }
}

export async function rpHub75SpiExample() {
    try {
        setupSpi();

        const renderBuffer = new ArrayBuffer(64 * 64 * 2);
        const view = new DataView(renderBuffer);

        const syncBuffer = buildSyncBuffer();
        const modesetBuffer = buildModesetBuffer();

        for (let i = 0; i < 64 * 64; i++) {
            view.setUint16(i * 2, i % 65535, true);
        }

        console.log(`Starting RP-HUB75 SPI loop (${syncBuffer.length} sync bytes + ${modesetBuffer.length} modeset bytes + rendered RGB565 frames)...`);

        // return;

        var j = 0;
        while (true) {
            sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);
            let pacer = sleep(33);

            for (let i = 0; i < 64 * 64; i++) {
                view.setUint16(i * 2, (j + i) % 65535, true);
            }
            j++;

            console.log(`frame: ${j}`);
            await pacer;
        }
    } catch (e) {
        console.log(e);
    }
}

solarSystemExample();
