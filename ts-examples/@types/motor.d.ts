declare module "motor" {

    type MoveDuration = { distance: number; } | { time: number; } | { };

    type MotorPins = {
        motA: number;
        motB: number;
        encA: number;
        encB: number;
    };

    type RegParams = {
        // All values must be integers, defaults to 0
        kp?: number;            // Proportional gain
        ki?: number;            // Integral gain
        kd?: number;            // Derivative gain
        kv?: number;            // Velocity feedforward
        ka?: number;            // Acceleration feedforward
        kc?: number;            // Constant feedforward
        maxIOut?: number;       // Maximum integral output
        unwindFactor?: number;  // Integral unwind factor (0 for no change)
    }

    class Motor {
        /**
         * Construc a new Motor instance
         * @param options Motor configuration
         * @note Units used in the circumference parameter determines the units used in the other methods
         */
        constructor(options: { pins: MotorPins, reg: RegParams, encTicks: number, circumference: number, frequency?: number });

        /**
         * Set the speed of the motor
         * @param speed Speed in units per second
         * @note The units are the same as used in the circumference parameter
         */
        setSpeed(speed: number): void;

        /**
         * Set the speed ramp of the motor
         * @param ramp Speed ramp in units per second squared
         * @note The units are the same as used in the circumference parameter
         */
        setRamp(ramp: number): void;

        /**
         * Set the end position tolerance for distance moves
         * @param tolerance
         */
        setEndPosTolerance(tolerance: number): void;

        /**
         * Move the motor
         * @param duration Duration of the movement
         * @note The units are the same as used in the circumference parameter
         * @note If the duration is not provided, the motor will move indefinitely
         */
        move(duration?: MoveDuration): Promise<void>;

        /**
         * Set raw power to the motor
         * @param pwm Power in the range -1023 to 1023
         * @note This switches the motor to unregulated mode
         */
        setRaw(power: number): void;

        /**
         * Stop the motor
         * @param brake If true, the motor will brake, otherwise it will coast to a stop
         */
        stop(brake?: boolean): Promise<void>;

        /**
         * Get the position of the motor
         * @note The units are the same as used in the circumference parameter
         * @returns The position of the motor
         */
        getPosition(): number;

        /**
         * Close the motor
         */
        close(): void;

        /**
         * Start logging motor data for reporting
         * @param everyNth Log every nth data point
         * @note The data is stored in memory and if the memory is full,
         *       may cause the program to crash. Use everyNth to limit
         *       the data points.
         * @note Previous data is not cleared until reportClear is called.
         */
        reportStart(everyNth: number): void;

        /**
         * Stop logging motor data
         */
        reportStop(): void;

        /**
         * Clear the report data
         */
        reportClear(): void;

        /**
         * Dump the report data to stdout
         */
        reportDump(): void;
    }
}
