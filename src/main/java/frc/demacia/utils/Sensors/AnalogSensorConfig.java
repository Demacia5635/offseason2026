package frc.demacia.utils.Sensors;

import com.ctre.phoenix6.CANBus;

import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;

/**
 * Base configuration for analog sensors.
 * 
 * <p>Extends BaseSensorConfig with analog-specific settings like offset calibration.</p>
 * 
 * @param <T> The concrete config type (for method chaining)
 */
public abstract class AnalogSensorConfig<T extends AnalogSensorConfig<T>> extends BaseSensorConfig<T> {
    public double offset = 0;

    /**
     * Constructor for analog port sensors.
     * 
     * @param channel Analog input port (0-3 on RoboRIO)
     * @param name Descriptive name for logging
     */
    public AnalogSensorConfig(int channel, String name) {
        super(channel, name);
    }

    /**
     * Constructor for CAN-based analog sensors.
     * 
     * @param id CAN bus ID
     * @param canbus CAN bus instance
     * @param name Descriptive name for logging
     */
    public AnalogSensorConfig(int id, Canbus canbus, String name) {
        super(id, canbus, name);
    }

    /**
     * Sets sensor zero offset.
     * 
     * <p>Offset is subtracted from raw reading for calibration.
     * Use to zero encoders or distance sensors at known positions.</p>
     * 
     * @param offset Offset value in sensor's native units
     * @return this config for method chaining
     */
    @SuppressWarnings("unchecked")
    public T withOffset(double offset) {
        this.offset = offset;
        return (T) this;
    }
}