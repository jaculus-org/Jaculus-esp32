import * as simpleradio from "simpleradio";
import * as wifi from "wifi";

/**
 * Presence test for SimpleRadio v2.
 *
 * On startup:
 * - uses MAC address in standard string form as device identity
 * - broadcasts it periodically
 * - listens for identities from other devices
 *
 * Run this on multiple boards in the same group and watch logs.
 */

const GROUP = 1;
const BROADCAST_PERIOD_MS = 1000;
const PEER_TIMEOUT_MS = 5000;

const myAddress = wifi.address();

interface PeerState {
    id: string;
    rssi: number;
    lastSeenMs: number;
}

const peers: { [address: string]: PeerState } = {};

function nowMs(): number {
    return Date.now();
}

function printPeers(): void {
    const now = nowMs();
    const entries: string[] = [];

    for (const address in peers) {
        const peer = peers[address];
        const age = now - peer.lastSeenMs;

        if (age > PEER_TIMEOUT_MS) {
            continue;
        }

        entries.push(`${peer.id}@${address} (RSSI ${peer.rssi}, ${age}ms ago)`);
    }

    if (entries.length === 0) {
        console.log("No peers visible yet.");
    }
    else {
        console.log(`Visible peers (${entries.length}): ${entries.join(", ")}`);
    }
}

function announce(): void {
    // Send our identity as standard MAC string.
    simpleradio.sendString(myAddress);
}

simpleradio.begin(GROUP);

console.log(`SimpleRadio presence test started`);
console.log(`Group: ${simpleradio.group()}, Radio Address: ${simpleradio.address()}, WiFi Address: ${myAddress}`);

simpleradio.on("string", (peerId, info) => {

    // Ignore our own packets (self-loop can happen on some setups).
    if (peerId === myAddress) {
        return;
    }

    const firstSeen = peers[info.address] === undefined;
    peers[info.address] = {
        id: peerId,
        rssi: info.rssi,
        lastSeenMs: nowMs(),
    };

    if (firstSeen) {
        console.log(`Discovered peer ${peerId} at ${info.address} (RSSI ${info.rssi})`);
    }
    else {
        console.log(`Updated peer ${peerId} at ${info.address} (RSSI ${info.rssi})`);
    }
});

announce();
setInterval(announce, BROADCAST_PERIOD_MS);
setInterval(printPeers, BROADCAST_PERIOD_MS);
