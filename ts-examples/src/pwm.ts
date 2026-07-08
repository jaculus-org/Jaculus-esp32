import { PWM } from "pwm";

/**
 * Example showing how to use the LEDC to control the brightness of an LED.
 */

const LED_PIN = 45;
const RESOLUTION = 10;
const MAX_DUTY = (1 << RESOLUTION) - 1;

const led = new PWM({
    pin: LED_PIN,
    frequency: 1000,
    resolution: RESOLUTION,
    duty: MAX_DUTY,
});

let duty = 0;
let step = 10;

let power = 3;

setInterval(() => {
    duty += step;
    if (duty >= MAX_DUTY) {
        duty = MAX_DUTY;
        step *= -1;
    }
    if (duty <= 0) {
        duty = 0;
        step *= -1;
    }
    led.setDuty(Math.pow(duty, power) / Math.pow(MAX_DUTY, power - 1));
}, 10);
