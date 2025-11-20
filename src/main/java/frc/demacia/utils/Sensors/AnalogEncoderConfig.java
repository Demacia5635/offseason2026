package frc.demacia.utils.Sensors;

public class AnalogEncoderConfig extends AnalogSensorConfig<AnalogEncoderConfig>{
    public double fullRange = 2 * Math.PI;
    public double minRange = 0;
    public double maxRange = 1.0;

    public AnalogEncoderConfig(int channel, String name) {
        super(channel, name);
        sensorType = AnalogEncoder.class;
    }

    public AnalogEncoderConfig withFullRange(double fullRange) {
        this.fullRange = fullRange * 2 * Math.PI;
        return this;
    }

    public AnalogEncoderConfig withRange(double minRange, double maxRange) {
        this.minRange = minRange;
        this.maxRange = maxRange;
        return this;
    }

    public AnalogEncoderConfig withMinRange(double minRange) {
        this.minRange = minRange;
        return this;
    }

    public AnalogEncoderConfig withMaxRange(double maxRange) {
        this.maxRange = maxRange;
        return this;
    }
}