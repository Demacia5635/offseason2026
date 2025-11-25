package frc.demacia.utils.Sensors;

import com.ctre.phoenix6.CANBus;

import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;

/**
 * Abstract base class for sensor configurations.
 * 
 * <p>Provides common configuration fields and methods for all sensor types including
 * digital inputs (DIO), analog inputs, and CAN-based sensors.</p>
 * 
 * <p><b>Supported Sensor Types:</b></p>
 * <ul>
 *   <li>Digital: Limit switches, beam breaks, encoders</li>
 *   <li>Analog: Potentiometers, distance sensors, analog encoders</li>
 *   <li>CAN: CANcoders, Pigeon2 IMU, color sensors</li>
 * </ul>
 * 
 * @param <T> The concrete config type (for method chaining)
 */
public abstract class BaseSensorConfig<T extends BaseSensorConfig<T>> {
    
    public Class<? extends SensorInterface> sensorType;
    
    public int id;
    public CANBus canbus;
    public String name;

    public int channel;
 
    public boolean isInverted;

    
    /**
     * @param name Descriptive name for logging (e.g., "FrontLimitSwitch")
     */
    public BaseSensorConfig(String name){
        this.name = name;
    }

    
    /**
     * Constructor for DIO or Analog port sensors.
     * 
     * @param channel DIO or Analog port number (0-9 typical)
     * @param name Descriptive name for logging (e.g., "FrontLimitSwitch")
     */
    public BaseSensorConfig(int channel, String name){
        this.channel = channel;
        this.name = name;
    }

    /**
     * Constructor for CAN-based sensors.
     * 
     * @param id CAN bus ID (1-63)
     * @param canbus CAN bus instance (Rio or CANivore)
     * @param name Descriptive name for logging
     */
    public BaseSensorConfig(int id, Canbus canbus, String name){
        this.id = id;
        this.canbus = canbus.canbus;
        this.name = name;
    }

    /**
     * Gets the sensor implementation class.
     * 
     * @return Class type of the sensor implementation
     */
    public Class<? extends SensorInterface> getSensorClass() {
        return sensorType;
    }

    /**
     * Sets sensor direction inversion.
     * 
     * <p>When inverted:</p>
     * <ul>
     *   <li>Digital sensors: true becomes false, false becomes true</li>
     *   <li>Encoders: positive rotation becomes negative</li>
     *   <li>Gyros: rotation direction reverses</li>
     * </ul>
     * 
     * @param isInverted true to invert sensor reading
     * @return this config for method chaining
     */
    @SuppressWarnings("unchecked")
    public T withInvert(boolean isInverted) {
        this.isInverted = isInverted;
        return (T) this;
    }
}
