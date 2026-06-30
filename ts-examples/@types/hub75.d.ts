declare module "hub75" {
    type Pixel = [number, number, number, number, number, number];
    type Pixels = Pixel[];

    class Hub75 {
        constructor(panelWidth?: number, panelHeight?: number, chainLength?: number);

        setBuffer(buffer: ArrayBuffer, size?: number, format?: number, clearPrev?: boolean): void;
        setBufferRaw(pixels: Pixels, format?: number, clearPrevious?: boolean): void;
        clear(): void;
        setBrightness(brightness: number): void;
        isInitialized(): boolean;
    }
}
