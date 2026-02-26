declare module "adc" {
    type Atten = number;

    const Attenuation: {
        readonly Db0: Atten;
        readonly Db2_5: Atten;
        readonly Db6: Atten;
        readonly Db12: Atten;
    };


    /**
     * Enable ADC on the given pin.
     * @param pin The pin to enable ADC on.
     */
    function configure(pin: number, attenuation?: Atten): void;

    /**
     * Read the value of the given pin.
     * @param pin The pin to read.
     * @returns The value of the pin (0-1023)
     */
    function read(pin: number): number;
}
