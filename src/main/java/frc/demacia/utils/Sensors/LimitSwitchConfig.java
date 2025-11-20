package frc.demacia.utils.Sensors;

public class LimitSwitchConfig extends BaseSensorConfig<LimitSwitchConfig>{

    /** 
     * Constructor
     * @param channel - DIO channel number
     * @param name - name of limit switch for logging
     */
    public LimitSwitchConfig(int channel, String name) {
        super(channel, name);
        sensorType = LimitSwitch.class;
    }
}
