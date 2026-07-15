declare module "i2c" {

    type I2CData = ArrayBuffer | Uint8Array | number[] | string | number | boolean;

    interface I2C {
        /**
         * Find an I2C interface by its pin.
         * @param pin The pin the I2C interface is connected to.
         * @returns The I2C interface, or undefined if not found.
         */
        find(pin: number): I2C | undefined;

        /**
         * Read from the given address.
         * @param address The address to read from.
         * @param quantity The number of bytes to read.
         * @returns The bytes read.
         */
        readFrom(address: number, quantity: number): Uint8Array;

        /**
         * Write to the given address.
         * @param address The address to write to.
         * @param buffer The data to write.
         */
        writeTo(address: number, buffer: I2CData): void;

        /**
         * Perform a write and read transaction.
         * @param address The address to write to.
         * @param buffer The data to write.
         * @param quantity The number of bytes to read.
         * @returns The bytes read.
         */
        writeRead(address: number, buffer: I2CData, quantity: number): Uint8Array;

        /**
         * Setup the I2C interface.
         * @param options The options to use when setting up the I2C interface.
         */
        setup(options: { scl?: number, sda?: number, bitrate?: number }): void;
    }

    const I2C1: I2C;
    const I2C2: I2C | undefined;
}
