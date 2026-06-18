declare module "onewire" {
    type OneWireData = ArrayBuffer | Uint8Array | number[] | string | number;

    class OneWire {
        /**
         * Create a OneWire instance on the specified data pin.
         * @param pin The specified pin
         */
        constructor(pin: number);

        /**
         * Reset the 1-Wire bus.
         * @return True if any devices are present, false otherwise.
         */
        reset(): boolean;

        /**
         * Read bytes from the bus.
         * @param count Number of bytes to read. Defaults to 1.
         * @return The bytes read.
         */
        read(count?: number): Uint8Array;

        /**
         * Write bytes to the bus.
         * @param data Data to write.
         */
        write(data: OneWireData): void;

        /**
         * Select a specific device by its 8-byte ROM.
         * @param rom Device ROM address.
         */
        select(rom: OneWireData): void;

        /**
         * Skip ROM selection and address all devices on the bus.
         */
        skip(): void;

        /**
         * Search devices on the bus.
         * @returns Array of 8-byte ROM addresses.
         */
        search(): Uint8Array[];

        /**
         * Compute Maxim/Dallas CRC-8 for the provided data.
         * @param data Data to checksum.
         * @returns CRC-8 value.
         */
        crc8(data: OneWireData): number;

        /**
         * Release bus resources.
         */
        close(): void;
    }
}
