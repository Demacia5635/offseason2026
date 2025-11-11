package frc.demacia.utils.Sensors;

import com.ctre.phoenix6.CANBus;

public abstract class AnalogSensorConfig<T extends AnalogSensorConfig<T>> extends BaseSensorConfig<T> {
    public double offset = 0;

    public AnalogSensorConfig(int channel, String name) {
        super(channel, name);
    }

    public AnalogSensorConfig(int id, CANBus canbus, String name) {
        super(id, canbus, name);
    }

    @SuppressWarnings("unchecked")
    public T withOffset(double offset) {
        this.offset = offset;
        return (T) this;
    }
}