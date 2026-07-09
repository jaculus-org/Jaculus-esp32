declare module "serial" {
    type Parity = number;
    type StopBits = number;
    type DataBits = number;

    const Parity: {
        None: Parity;
        Even: Parity;
        Odd: Parity;
    };

    const StopBits: {
        One: StopBits;
        Two: StopBits;
    };

    const DataBits: {
        Five: DataBits;
        Six: DataBits;
        Seven: DataBits;
        Eight: DataBits;
    };

    interface SerialOptions {
        tx: number;
        rx: number;
        baudRate: number;
        parity?: Parity;
        stopBits?: StopBits;
        dataBits?: DataBits;
        rxBufferSize?: number;
        txBufferSize?: number;
        invertTx?: boolean;
        invertRx?: boolean;
    }

    interface Serial {
        /**
         * Configure and open the serial port. If the port is already open it is
         * closed and reconfigured.
         * @param options Serial port configuration.
         */
        setup(options: SerialOptions): void;

        /**
         * Wait for and consume a single byte from the receive buffer.
         * If no data is available the promise resolves once a byte is received.
         * @returns A promise that resolves with a one-byte Uint8Array.
         */
        get(): Promise<Uint8Array>;

        /**
         * Wait for and consume all currently buffered received data.
         * If no data is available the promise resolves once data is received.
         * @returns A promise that resolves with the received data as a Uint8Array.
         */
        read(): Promise<Uint8Array>;

        /**
         * Write data to the UART.
         * @param data Data to transmit.
         */
        write(data: Uint8Array): void;

        /**
         * Block until all pending TX data has been transmitted.
         */
        flush(): void;

        /**
         * Close the serial port and release the UART driver.
         * Any pending get or read promises are rejected.
         */
        close(): void;
    }

    const Serial1: Serial;
    const Serial2: Serial | undefined;
    const Serial3: Serial | undefined;
}
