import { Font, Format, Renderer, Texture } from 'renderer';
import { Collection, Circle, LineSegment, Matrix2D, Polygon, Rectangle, RegularPolygon } from 'shapes';
import { setupSpi, buildSyncBuffer, buildModesetBuffer, sendRpHub75Frame } from './renderer/spiSender.js';

const PANEL_WIDTH = 64;
const PANEL_HEIGHT = 64;
const MAX_PIXELS = PANEL_WIDTH * PANEL_HEIGHT;
const BUFFER_SIZE_BYTES = MAX_PIXELS * 3;

export async function solarSystemExample() {
    setupSpi();

    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT, Format.RGB_888, 0);
    const renderBuffer = new ArrayBuffer(BUFFER_SIZE_BYTES);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_888);

    const sunCollection = new Collection({ x: 0, y: 0, z: 0 });
    sunCollection.setPivot(32, 32);

    const sun = new Circle({
        x: 32, y: 32,
        radius: 8,
        color: 0xffff00,
        fill: true
    });
    sunCollection.add(sun);

    const earthCollection = new Collection({ x: 0, y: 0, z: 5 });
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

    const moonCollection = new Collection({ x: 0, y: 0, z: 10 });
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
        renderer.render(sunCollection, renderBuffer, true);
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

    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT, Format.RGB_888, 0);
    const renderBuffer = new ArrayBuffer(BUFFER_SIZE_BYTES);
    const font = new Font();
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_888);

    const scene = new Collection({ x: 0, y: 0, z: 10 });
    const rect = new Rectangle({ x: 0, y: 0, width: 21, height: 8, color: 0x222222, fill: true });
    scene.add(rect);

    while (true) {
        renderer.render(scene, renderBuffer, true);
        renderer.drawText(renderBuffer, "rot0", 0, 0, font, 0xff0000, false, 0);
        renderer.drawText(renderBuffer, "rot1", 0, 0, font, 0x00ff00, false, 1);
        renderer.drawText(renderBuffer, "rot2", 0, 0, font, 0x0000ff, false, 2);
        renderer.drawText(renderBuffer, "rot3", 0, 0, font, 0xffff00, false, 3);
        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);
        await sleep(1);
    }
}

export async function textureExample() {
    setupSpi();

    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT, Format.RGB_888, -1);
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

    const scene = new Collection({ x: 0, y: 0, z: 0 });
    const rect = new Rectangle({ x: 30, y: 30, width: 30, height: 25, color: 0xFFFFFF, fill: true });
    rect.setTexture(texture);
    rect.setTextureScale(2, 2);
    scene.add(rect);

    while (true) {
        rect.rotate(1);
        renderer.render(scene, renderBuffer, true);
        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);
        await sleep(1);
    }
}

export async function removeExample() {
    setupSpi();

    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT, Format.RGB_888, 0);
    const renderBuffer = new ArrayBuffer(BUFFER_SIZE_BYTES);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_888);

    const scene = new Collection({ x: 0, y: 0, z: 0 });

    const colors = [0xff0000, 0x00ff00, 0x0000ff, 0xffff00, 0xff00ff];
    const circles = colors.map((color, i) => new Circle({
        x: 10 + i * 11, y: 32,
        radius: 5,
        color,
        fill: true
    }));
    circles.forEach(circle => scene.add(circle));

    let frame = 0;
    let next = 0;

    while (true) {
        if (frame > 0 && frame % 20 === 0) {
            if (next < circles.length) {
                scene.remove(circles[next]);
                next++;
            } else {
                circles.forEach(circle => scene.add(circle));
                next = 0;
            }
        }

        renderer.render(scene, renderBuffer, true);
        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);
        frame++;
        await sleep(1);
    }
}

export async function transformExample() {
    setupSpi();

    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT, Format.RGB_888, 0);
    const renderBuffer = new ArrayBuffer(BUFFER_SIZE_BYTES);
    const syncBuffer = buildSyncBuffer();
    const modesetBuffer = buildModesetBuffer(PANEL_WIDTH, Format.RGB_888);

    const texture = new Texture();
    texture.load("/data/data/renderer/brick.bmp");

    const scene = new Collection({ x: 0, y: 0, z: 0 });

    const blob = new Circle({ x: 16, y: 16, radius: 10, color: 0x00ffff, fill: true });
    blob.setTexture(texture);
    blob.setFixTexture(true);
    blob.setTextureScale(2, 2);
    scene.add(blob);

    const panel = new Rectangle({ x: 40, y: 6, width: 12, height: 12, color: 0xff8800, fill: true });
    scene.add(panel);

    const skewed = new Rectangle({ x: 0, y: 0, width: 28, height: 14, color: 0xff00ff, fill: true });
    skewed.setTexture(texture);
    skewed.setTextureScale(2, 2);
    scene.add(skewed);

    let t = 0;

    while (true) {
        t += 0.05;

        blob.setScale(1.5 + Math.sin(t), 1.5 + Math.cos(t));

        const s = 1 + 0.3 * Math.sin(t * 2);
        panel.setScale(s, s);

        const shear = 0.4 * Math.sin(t);
        const matrix: Matrix2D = {
            a: 1, b: 0,
            c: shear, d: 1,
            e: 20, f: 40,
        };
        if (t < 10) {
            skewed.setTransformationMatrix(matrix);
        } else if (t >= 10 && t < 15) {
            skewed.clearTransformationMatrix();
        }

        renderer.render(scene, renderBuffer, true);
        sendRpHub75Frame(syncBuffer, modesetBuffer, renderBuffer);
        await sleep(1);
    }
}

solarSystemExample();
