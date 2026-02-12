declare module "spi" {
    type SPIData = ArrayBuffer | Uint8Array | number[] | string | number;

    interface SPI {
        /**
         * Send data and return received bytes
         * @param data Data to send.
         * @param cs Chip-select pin.
         */
        send(data: SPIData, cs: number): Uint8Array;

        /**
         * Write data
         * @param data Data to write.
         * @param cs Chip-select pin.
         */
        write(data: SPIData, cs: number): void;

        /**
         * Perform a write and read transaction.
         * @param data Data to write.
         * @param cs Chip-select pin.
         * @param rxLength Number of bytes to read.
         * @param qio Whether to use quad SPI mode.
         * @returns The bytes read.
         */
        transfer(data: SPIData, cs: number, rxLength: number, qio: boolean): Uint8Array;

        /**
         * Configure SPI bus and device pins/options.
         * Accepts either mosi/miso or data0/data1 for pins of the first two data lines;
         * data2/data3 can be used for quad SPI with qio=true in transfer() calls.
         */
        setup(options: {
            sck?: number;
            mosi?: number;
            miso?: number;
            data0?: number;
            data1?: number;
            data2?: number;
            data3?: number;
            baud?: number;
            mode?: number;
            order?: "lsb" | "msb";
        }): void;
    }

    const SPI1: SPI;
    const SPI2: SPI;
    const SPI3: SPI | undefined;
}
