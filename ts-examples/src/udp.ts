import { bytesToStr } from "./bytes.js";
import { UdpSocket } from "udp";
import { currentIp } from "wifi";

/**
 * Example UDP echo server.
 */

while (!currentIp()) {
    console.log("Waiting for network...");
    await sleep(1000);
}

const PORT = 1234;

const socket = new UdpSocket({
    address: "0.0.0.0",
    port: PORT,
    onError: () => {
        console.log("Socket error");
    },
    onReadable: (count) => {
        console.log(`${count} datagrams available`);
        const msg = socket.read();
        if (msg) {
            console.log(`Received: ${bytesToStr(msg)} from ${msg.address}:${msg.port}`);
            socket.write(msg, msg.address, PORT);
        }
    }
});

setInterval(() => {
    console.log(`Listening on ${currentIp()}:${PORT}`);
}, 5000);
