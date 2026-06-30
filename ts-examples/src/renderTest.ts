import { Font, Renderer, Texture } from 'renderer';
import { Collection, Circle, Rectangle } from 'shapes';
import { Format } from './constants.js';
import { setupSpi, buildSyncBuffer, buildModesetBuffer, sendRpHub75Frame } from './spiSender.js';
import { rectangleExample } from './rp-hub75-spi.js';

const PANEL_WIDTH = 64;
const PANEL_HEIGHT = 64;
const MAX_PIXELS = PANEL_WIDTH * PANEL_HEIGHT;
const BUFFER_SIZE_BYTES = MAX_PIXELS * 3;

export async function solarSystemExample() {
    setupSpi();

    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT);
    const renderBuffer = new ArrayBuffer(BUFFER_SIZE_BYTES);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_888);

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
    let angle = 0;

    console.log(`Starting RP-HUB75 SPI loop (${syncBuffer.length} sync bytes + ${modesetBuffer.length} modeset bytes + rendered RGB565 frames)...`);

    while (true) {
        earthCollection.rotate(1.5);
        moonCollection.rotate(3);
        renderer.render(sunCollection, renderBuffer, true, Format.RGB_888);
        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);
        angle += 0.15;
        await sleep(1);
    }
}

export async function rotatedGridExample() {
    setupSpi();

    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT);
    const renderBuffer = new ArrayBuffer(BUFFER_SIZE_BYTES);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_888);

    const scene = new Collection({ x: 0, y: 0, color: [0, 0, 0, 255], z: 0 });
    for (let i = 0; i < 14; i++) {
        for (let j = 0; j < 5; j++) {
            let rect = new Rectangle({ x: i * 5, y: j * 5, width: 4, height: 4, color: [12, 54, 160, 255], fill: true });
            rect.rotate(30);
            scene.add(rect);
        }
    }

    while (true) {
        renderer.render(scene, renderBuffer, true, Format.RGB_888, -1);
        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);
        await sleep(1);
    }
}

export async function gridExample() {
    setupSpi();

    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT);
    const renderBuffer = new ArrayBuffer(BUFFER_SIZE_BYTES);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_888);

    const scene = new Collection({ x: 0, y: 0, color: [0, 0, 0, 255], z: 0 });
    for (let i = 0; i < 16; i++) {
        for (let j = 0; j < 16; j++) {
            let rect = new Rectangle({ x: i * 4, y: j * 4, width: 3, height: 3, color: [12, 54, 160, 255] });
            scene.add(rect);
        }
    }

    while (true) {
        renderer.render(scene, renderBuffer, true, Format.RGB_888, -1);
        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);
        await sleep(1);
    }
}


export async function rectExample() {
    setupSpi();

    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT);
    const renderBuffer = new ArrayBuffer(BUFFER_SIZE_BYTES);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_888);

    const scene = new Collection({ x: 0, y: 0, color: [0, 0, 0, 255], z: 0 });
    const rect = new Rectangle({ x: 0, y: 0, width: 64, height: 64, color: [255, 255, 255, 255], fill: true });
    scene.add(rect);


    while (true) {
        rect.rotate(2);
        renderer.render(scene, renderBuffer, true, Format.RGB_888, -1);
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
        const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_888);

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

export async function textExample() {
    setupSpi();

    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT);
    const renderBuffer = new ArrayBuffer(BUFFER_SIZE_BYTES);
    const font = new Font();
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_888);

    while (true) {
        renderer.drawText(renderBuffer, "Testing baf, test, #!%$@", 0, 0, font, [255, 0, 0, 255], true, Format.RGB_888, -1);
        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);
        await sleep(1);
    }
}

export async function textureExample() {
    setupSpi();

    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT);
    const renderBuffer = new ArrayBuffer(BUFFER_SIZE_BYTES);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_888);

    const texture = new Texture();
    const success = texture.load("/data/data/brick.bmp");
    if (success) {
        texture.setWrapMode("repeat");
        console.log(`Texture loaded: ${texture.getWidth()}x${texture.getHeight()}`);
    } else {
        console.log("Failed to load texture BMP");
    }

    const scene = new Collection({ x: 0, y: 0, color: [0, 0, 0, 255], z: 0 });
    const rect = new Rectangle({ x: 30, y: 30, width: 30, height: 25, color: [12, 54, 160, 255], fill: true });
    rect.setTexture(texture);
    rect.setTextureScale(2, 2);
    rect.setFixTexture(true);
    scene.add(rect);

    while (true) {
        rect.rotate(1);
        renderer.render(scene, renderBuffer, true, Format.RGB_888, -1);
        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);
        await sleep(1);
    }
}

async function generatedScene() {
    setupSpi();
    const renderer = new Renderer(64, 64);
    const renderBuffer = new ArrayBuffer(64 * 64 * 3);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(64, Format.RGB_888);
    const scene = new Collection({ x: 0, y: 0, color: [0, 0, 0, 255] });

    const rectangle_d3t = new Rectangle({
        x: 24, y: 20,
        color: [241, 9, 6, 255],
        width: 26, height: 16,
        fill: true
    });
    rectangle_d3t.rotate(18);
    scene.add(rectangle_d3t);
    console.log("Rendering");

    while (true) {
        renderer.render(scene, renderBuffer, true, Format.RGB_888, -1);
        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);
        await sleep(100);
    }
}

rectangleExample();
