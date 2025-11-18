package frc.demacia.utils.Sensors;

public class DigitalEncoderConfig extends BaseSensorConfig<DigitalEncoderConfig>{
    public double scalar = 1;
    public double minRange = 0;
    public double maxRange = 1;
    public double frequency = 1000;
    public double connectedFrequencyThreshold = 100;
    public double offset = 0;

    public DigitalEncoderConfig(int channel, String name) {
        super(channel, name);
        sensorType = DigitalEncoder.class;
    }

    public DigitalEncoderConfig withScalar(double scalar) {
        this.scalar = scalar;
        return this;
    }

    public DigitalEncoderConfig withRange(double minRange, double maxRange) {
        this.minRange = minRange;
        this.maxRange = maxRange;
        return this;
    }

    public DigitalEncoderConfig withMinRange(double minRange) {
        this.minRange = minRange;
        return this;
    }

    public DigitalEncoderConfig withMaxRange(double maxRange) {
        this.maxRange = maxRange;
        return this;
    }

    public DigitalEncoderConfig withFrequency(double frequency) {
        this.frequency = frequency;
        return this;
    }

    public DigitalEncoderConfig withOffset(double offset) {
        this.offset = offset;
        return this;
    }
}