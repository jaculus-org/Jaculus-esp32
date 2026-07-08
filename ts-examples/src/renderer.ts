import { Font, Format, Renderer, Texture } from 'renderer';
import { Collection, Circle, Rectangle } from 'shapes';
import { setupSpi, buildSyncBuffer, buildModesetBuffer, sendRpHub75Frame } from './renderer/spiSender.js';

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

    const sunCollection = new Collection({ x: 0, y: 0, color: 0x000000, z: 0 });
    sunCollection.setPivot(32, 32);

    const sun = new Circle({
        x: 32, y: 32,
        radius: 8,
        color: 0xffff00,
        fill: true
    });
    sunCollection.add(sun);

    const earthCollection = new Collection({ x: 0, y: 0, color: 0x000000, z: 5 });
    earthCollection.setPivot(32, 32);

    const earth = new Circle({
        x: 32 + 20, y: 32,
        radius: 4,
        color: 0x0096ff,
        fill: true,
        z: 10
    });
    earthCollection.add(earth);

    const alien = new Circle({
        x: 32 - 20, y: 32,
        radius: 4,
        color: 0x00ff64,
        fill: true,
        z: 10
    });
    earthCollection.add(alien);

    const moonCollection = new Collection({ x: 0, y: 0, color: 0x000000, z: 10 });
    moonCollection.setPivot(52, 32);

    const moon = new Circle({
        x: 32 + 20 + 8, y: 32,
        radius: 2,
        color: 0xc8c8c8,
        fill: true,
        z: 10
    });
    moonCollection.add(moon);

    earthCollection.add(moonCollection);
    sunCollection.add(earthCollection);

    const earthOrbit = new Circle({
        x: 32, y: 32,
        radius: 20,
        color: 0x646464,
        fill: false,
        z: 1
    });
    sunCollection.add(earthOrbit);

    const moonOrbit = new Circle({
        x: 32 + 20, y: 32,
        radius: 8,
        color: 0x646464,
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

export async function rawDataExample() {
    try {
        setupSpi();

        const renderBuffer = new ArrayBuffer(64 * 64 * 2);
        const view = new DataView(renderBuffer);

        const syncBuffer = buildSyncBuffer();
        const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_565_LITTLE);

        for (let i = 0; i < 64 * 64; i++) {
            view.setUint16(i * 2, i % 65535, true);
        }

        console.log(`Starting RP-HUB75 SPI loop (${syncBuffer.length} sync bytes + ${modesetBuffer.length} modeset bytes + rendered RGB565 frames)...`);


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
        renderer.drawText(renderBuffer, "Testing baf, test, #!%$@", 0, 0, font, 0xff0000, true, Format.RGB_888, -1);
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
    const success = texture.load("/data/data/renderer/brick.bmp");
    if (success) {
        texture.setWrapMode("repeat");
        console.log(`Texture loaded: ${texture.getWidth()}x${texture.getHeight()}`);
    } else {
        console.log("Failed to load texture BMP");
    }

    const scene = new Collection({ x: 0, y: 0, color: 0x000000, z: 0 });
    const rect = new Rectangle({ x: 30, y: 30, width: 30, height: 25, color: 0x0c36a0, fill: true });
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

solarSystemExample();
