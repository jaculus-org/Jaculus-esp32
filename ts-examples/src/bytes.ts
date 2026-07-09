/**
 * Simple byte <-> string conversion helpers.
 */

export function strToBytes(s: string): Uint8Array {
    const bytes = new Uint8Array(s.length);
    for (let i = 0; i < s.length; i++) {
        bytes[i] = s.charCodeAt(i);
    }
    return bytes;
}

export function bytesToStr(b: Uint8Array | ArrayBuffer): string {
    const bytes = b instanceof ArrayBuffer ? new Uint8Array(b) : b;
    return String.fromCharCode.apply(null, bytes as unknown as number[]);
}

export function bytesEqual(a: Uint8Array, b: Uint8Array): boolean {
    if (a.length !== b.length) {
        return false;
    }
    for (let i = 0; i < a.length; i++) {
        if (a[i] !== b[i]) {
            return false;
        }
    }
    return true;
}
