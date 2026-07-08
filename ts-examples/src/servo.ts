import * as adc from "adc";
import { PWM } from "pwm";

/**
 * Example showing how to control servos using the LEDC peripheral.
 */

const INPUT_PIN = 1;
const SERVO_PIN = 17;
const RESOLUTION = 12;
const MAX_DUTY = (1 << RESOLUTION) - 1;

const servo = new PWM({
    pin: SERVO_PIN,
    frequency: 50,
    resolution: RESOLUTION,
    duty: MAX_DUTY,
});


setInterval(() => {
    const value = adc.read(INPUT_PIN);

    // map the value from 0-1023 to 1-2ms
    const ms = (value / 1023) + 1;

    // convert to a duty cycle
    const duty = (ms / 20) * MAX_DUTY;

    servo.setDuty(duty);
}, 20);
