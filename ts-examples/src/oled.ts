import { I2C1 } from 'i2c';
import { Renderer } from 'renderer';
import { Format } from './constants.js';
import {
    Collection, Rectangle, Circle, RegularPolygon,
    Polygon, Point, LineSegment
} from 'shapes';

const PANEL_WIDTH = 128;
const PANEL_HEIGHT = 64;
const ADDR_RIGHT = 0x3C;
const PIN_SDA = 14;
const PIN_SCL = 13;

const MAX_PIXELS = PANEL_WIDTH * PANEL_HEIGHT;
const BUFFER_SIZE_BYTES = MAX_PIXELS;

function sendCommands(address: number, commands: number[]) {
    const payload = new Uint8Array([0x00, ...commands]);
    I2C1.writeTo(address, payload);
}

function initOLED(address: number) {
    I2C1.setup({ sda: PIN_SDA, scl: PIN_SCL, bitrate: 1000000 });
    sendCommands(address, [
        0xAE, 0x20, 0x00, 0x21, 0, 127, 0x22, 0, 7, 0x8D, 0x14, 0xAF
    ]);
}

function sendBufferToOLED(address: number, oledBuffer: Uint8Array) {
    sendCommands(address, [0x21, 0, 127, 0x22, 0, 7]);

    for (let i = 0; i < oledBuffer.length; i += 128) {
        const chunk = oledBuffer.subarray(i, i + 128);
        const payload = new Uint8Array(chunk.length + 1);
        payload[0] = 0x40;
        payload.set(chunk, 1);
        I2C1.writeTo(address, payload);
    }
}

export async function shapeExample() {
    initOLED(ADDR_RIGHT);

    const renderer = new Renderer(PANEL_WIDTH, PANEL_HEIGHT);
    const renderBuffer = new Uint8Array(BUFFER_SIZE_BYTES);
    const oledBuffer = new Uint8Array(1024);

    const scene = new Collection({ x: 0, y: 0, color: [0, 0, 0, 255] });

    const line1 = new LineSegment({ x: 0, y: 20, x2: 63, y2: 35, color: [255, 255, 255, 255] });
    scene.add(line1);

    const rect = new Rectangle({ x: 6, y: 6, width: 6, height: 6, color: [255, 255, 255, 255], fill: true });
    scene.add(rect);

    const circle = new Circle({ x: 18, y: 6, radius: 5, color: [255, 255, 255, 255], fill: true });
    scene.add(circle);

    const hexagon = new RegularPolygon({ x: 48, y: 48, radius: 10, sides: 6, color: [255, 255, 255, 255], fill: true });
    scene.add(hexagon);

    const pentagon = new RegularPolygon({ x: 48, y: 16, radius: 8, sides: 5, color: [255, 255, 255, 255], fill: true });
    scene.add(pentagon);

    const polygon = new Polygon({ x: 16, y: 48, color: [255, 255, 255, 255], vertices: [[0, 0], [10, 5], [5, 15], [0, 10]], fill: true });
    scene.add(polygon);

    const line2 = new LineSegment({ x: 32, y: 0, x2: 20, y2: 40, color: [255, 255, 255, 255] });
    scene.add(line2);

    const lineRect = new Rectangle({ x: 25, y: 20, width: 8, height: 5, color: [255, 255, 255, 255], fill: true });
    scene.add(lineRect);

    const point = new Point({ x: 32, y: 42, color: [255, 255, 255, 255] });
    scene.add(point);

    console.log("Starting Monochrome OLED Render Loop (5-bytes per pixel)...");

    while (true) {
        renderer.render(scene, renderBuffer.buffer, true, Format.MONOCHROME);

        oledBuffer.fill(0);

        for (let y = 0; y < PANEL_HEIGHT; y++) {
            for (let x = 0; x < PANEL_WIDTH; x++) {
                const color = renderBuffer[x + y * PANEL_WIDTH];
                if (color !== 1) {
                    continue;
                }

                const page = Math.floor(y / 8);
                const bit = y % 8;
                const bufferIndex = x + (page * PANEL_WIDTH);

                oledBuffer[bufferIndex] |= (1 << bit);
            }
        }

        sendBufferToOLED(ADDR_RIGHT, oledBuffer);

        await sleep(100);
    }
}

shapeExample();
