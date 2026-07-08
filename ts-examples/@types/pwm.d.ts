declare module "pwm" {
    type PWMOptions = {
        pin: number,
        frequency: number,
        resolution?: number,
        duty?: number,
    };

    /**
     * PWM output bound to a fixed timer configuration.
     */
    class PWM {
        constructor(options: PWMOptions);
        readonly pin: number;
        readonly timer: number;
        readonly channel: number;
        readonly frequency: number;
        readonly resolution: number;

        /**
         * Update the duty cycle.
         * @param duty Duty cycle value in the range allowed by the configured resolution.
         */
        setDuty(duty: number): void;

        /**
         * Release the reserved timer and channel resources.
         */
        close(): void;
    }

    /**
     * PWM output with independently adjustable timer settings.
     */
    class VariablePWM {
        constructor(options: PWMOptions);
        readonly pin: number;
        readonly timer: number;
        readonly channel: number;
        readonly frequency: number;
        readonly resolution: number;

        /**
         * Update the duty cycle.
         * @param duty Duty cycle value in the range allowed by the configured resolution.
         */
        setDuty(duty: number): void;

        /**
         * Update the output frequency.
         * @param frequency New frequency in Hz.
         */
        setFrequency(frequency: number): void;

        /**
         * Update the PWM resolution.
         * @param resolution New resolution in bits.
         */
        setResolution(resolution: number): void;

        /**
         * Release the reserved timer and channel resources.
         */
        close(): void;
    }
}
