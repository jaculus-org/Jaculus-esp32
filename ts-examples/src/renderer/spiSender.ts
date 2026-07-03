import { SPI2 } from 'spi';
import { Format } from 'renderer';

const PIN_SCK = 3;
const PIN_CS = 1;
const SPI_BAUD = 2_000_000;
const SPI_MODE = 0;
const MODE_MAGIC = 0xfb;

const SYNC_WORDS = [
    0xac92, 0x3bca, 0x41bf, 0x393d, 0xa74a, 0xae01, 0x155d, 0xfb70, 0xf681, 0x2f6d, 0x4931, 0x0fa3, 0x77bf, 0xd756, 0x26f9, 0x4eb6,
];

export function setupSpi() {
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

export function buildSyncBuffer() {
    const buffer = new Uint8Array(SYNC_WORDS.length * 2);
    const view = new DataView(buffer.buffer);

    for (let i = 0; i < SYNC_WORDS.length; i++) {
        view.setUint16(i * 2, SYNC_WORDS[i], true);
    }

    return buffer;
}

export function buildModesetBuffer(width: number, format: Format) {
    const buffer = new Uint8Array(8);
    const view = new DataView(buffer.buffer);

    view.setUint8(0, MODE_MAGIC);
    view.setUint8(1, 0);
    view.setUint8(2, format);
    view.setUint8(3, 255);
    view.setUint16(4, width, true);

    return buffer;
}

export function sendRpHub75Frame(syncBuffer: Uint8Array, modesetBuffer: Uint8Array, frameBuffer: ArrayBuffer) {
    SPI2.transfer(syncBuffer, PIN_CS, 0, true);
    SPI2.transfer(modesetBuffer, PIN_CS, 0, true);

    SPI2.transfer(frameBuffer, PIN_CS, 0, true);
}
