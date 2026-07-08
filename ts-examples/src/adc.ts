import * as adc from "adc";
import { PWM } from "pwm";

/**
 * Example showing how to use the ADC to control the brightness of an LED.
 */

const INPUT_PIN = 1;
const LED_PIN = 45;
const RESOLUTION = 10;
const MAX_DUTY = (1 << RESOLUTION) - 1;

const led = new PWM({
    pin: LED_PIN,
    frequency: 1000,
    resolution: RESOLUTION,
    duty: MAX_DUTY,
});

adc.configure(INPUT_PIN);

let power = 3;

setInterval(() => {
    const value = adc.read(INPUT_PIN);
    led.setDuty(Math.pow(value, power) / Math.pow(1023, power - 1) * MAX_DUTY / 1023);
}, 10);
