package frc.Demacia.utils.Sensors;

import frc.Demacia.utils.Motors.BaseMotorConfig.Canbus;

/** Add your docs here. */
public class CancoderConfig {
    public int id;                  // Canbus ID
    public Canbus canbus;
    public String name; 
    public double offset = 0;
    public boolean inverted = false; // if to invert cancoderr
    /** 
     * Constructor
     * @param id - canbus ID
     * @param canbus - Name of canbus
     * @param name - name of Cancoder for logging
     */
    public CancoderConfig(int id, Canbus canbus, String name) {
        this.id = id;
        this.canbus = canbus;
        this.name = name;
    }

    public CancoderConfig withOffset(double offset) {
        this.offset = offset;
        return this;
    }

    /** 
     * @param invert
     * @return CancoderConfig
     */
    public CancoderConfig withInvert(boolean invert) {
        this.inverted = invert;
        return this;
    }
}