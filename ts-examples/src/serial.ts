import { Serial1 } from "serial";
import { bytesToStr, strToBytes } from "./bytes.js";

Serial1.setup({
    tx: 2,
    rx: 10,
    baudRate: 115200
});

setInterval(async () => {
    Serial1.write(strToBytes("Hello, world!\n"));
}, 1000);

while (true) {
    let data = await Serial1.read();
    console.log("Received: " + bytesToStr(data));
}
