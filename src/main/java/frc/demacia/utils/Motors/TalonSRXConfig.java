package frc.demacia.utils.Motors;


/** 
 * Class to hold all Talon FX/SRX configuration
 * Applicable to Phoenix 6
 *  */
public class TalonSRXConfig extends BaseMotorConfig<TalonSRXConfig> {

    /** 
     * Constructor
     * @param id - canbus ID
     * @param canbus - Name of canbus
     * @param name - name of motor for logging
     */
    public TalonSRXConfig(int id, String name) {
        super(id, name);
        motorClass = MotorControllerType.TalonSRX;
    }

    public TalonSRXConfig(int id, String name, TalonConfig config) {
        super(id, name);
        copyBaseFields(config);
    }
}