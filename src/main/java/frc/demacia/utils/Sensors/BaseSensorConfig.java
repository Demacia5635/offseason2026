package frc.demacia.utils.Sensors;

import com.ctre.phoenix6.CANBus;

public abstract class BaseSensorConfig<T extends BaseSensorConfig<T>> {
    
    public Class<? extends SensorInterface> sensorType;
    
    public int id;
    public CANBus canbus;
    public String name;

    public int channel;
 
    public boolean isInverted;

    /**
     * Constructor for DIO/Analog sensors
     */
    public BaseSensorConfig(int channel, String name){
        this.channel = channel;
        this.name = name;
    }

    /**
     * Constructor for CAN sensors
     */
    public BaseSensorConfig(int id, CANBus canbus, String name){
        this.id = id;
        this.canbus = canbus;
        this.name = name;
    }

    public Class<? extends SensorInterface> getSensorClass() {
        return sensorType;
    }

    /**
     * Set sensor inversion
     */
    @SuppressWarnings("unchecked")
    public T withInvert(boolean isInverted) {
        this.isInverted = isInverted;
        return (T) this;
    }
}
