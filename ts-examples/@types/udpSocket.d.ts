declare module "udp" {

    interface UdpBuffer extends ArrayBuffer {
        address: string;
        port: number;
    }

    class UdpSocket {
        /**
         * Creates a UDP socket and binds it to the specified address and port.
         * @param options The socket options.
         */
        constructor(options: {
            address?: string;
            port?: number;
            onReadable?: (bytesAvailable: number) => void;
            onError?: (error: Error) => void;
        });

        /**
         * Reads a single datagram from the socket.
         * @returns The datagram buffer with `address` and `port` properties,
         *          or `null` if no datagram is available.
         */
        read(): UdpBuffer | null;

        /**
         * Sends a datagram to the specified address and port.
         * @param data The data to send.
         * @param address The destination IP address.
         * @param port The destination port.
         */
        write(data: ArrayBuffer, address: string, port: number): void;

        /**
         * Closes the socket.
         */
        close(): void;
    }
}
