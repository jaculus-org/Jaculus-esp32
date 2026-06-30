declare module "simpleradio" {
    type PacketDataType = "number" | "string" | "keyvalue" | "blob";
    type BlobData = ArrayBuffer | Uint8Array | number[];

    interface PacketInfo {
        group: number;
        address: string;
        rssi: number;
    }

    /**
     * Initialize the radio.
     * @param group The radio group to use.
     */
    function begin(group: number): void;

    /**
     * Set the radio group.
     * @param group The radio group to use, between 0 and 255 inclusive.
     */
    function setGroup(group: number): void;

    /**
     * Get current radio group
     * @returns ID of the current group
     */
    function group(): number;

    /**
     * Get the local device address.
     * @returns the local device address. Only works after begin() is called.
     */
    function address(): string;

    /**
     * Send a string.
     * @param str The string to send.
     */
    function sendString(str: string): void;

    /**
     * Send a number.
     * @param num The number to send.
     */
    function sendNumber(num: number): void;

    /**
     * Send a key-value pair.
     * @param key The key to send.
     * @param value The number to send.
     */
    function sendKeyValue(key: string, value: number): void;

    /**
     * Send a binary blob.
     * @param data The binary data to send.
     */
    function sendBlob(data: BlobData): void;

    /**
     * Register a callback for a packet type.
     * @param type The packet type to register for.
     * @param callback The callback to register.
     */
    function on(type: "number", callback: (num: number, info: PacketInfo) => void): void;

    /**
     * Register a callback for a packet type.
     * @param type The packet type to register for.
     * @param callback The callback to register.
     */
    function on(type: "string", callback: (str: string, info: PacketInfo) => void): void;

    /**
     * Register a callback for a packet type.
     * @param type The packet type to register for.
     * @param callback The callback to register.
     */
    function on(type: "keyvalue", callback: (key: string, value: number, info: PacketInfo) => void): void;

    /**
     * Register a callback for a packet type.
     * @param type The packet type to register for.
     * @param callback The callback to register.
     */
    function on(type: "blob", callback: (data: Uint8Array, info: PacketInfo) => void): void;

    /**
     * Unregister a callback for a packet type.
     * @param type The packet type to unregister for.
     */
    function off(type: PacketDataType): void;

    /**
     * Stop the radio.
     */
    function end(): void;
}
