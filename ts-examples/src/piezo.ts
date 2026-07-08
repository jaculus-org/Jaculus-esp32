import { VariablePWM } from "pwm";

/**
 * Example showing how to use the LEDC to control the frequency of a piezo.
 */

const PIEZO_PIN = 18;

const piezo = new VariablePWM({
    pin: PIEZO_PIN,
    frequency: 1000,
    duty: 512,
});

let frequency = 1000;
let step = 10;

setInterval(() => {
    frequency += step;
    if (frequency >= 2000) {
        frequency = 2000;
        step *= -1;
    }
    if (frequency <= 1000) {
        frequency = 1000;
        step *= -1;
    }
    piezo.setFrequency(frequency);
}, 10);
