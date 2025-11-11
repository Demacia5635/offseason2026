package frc.demacia.utils.Motors;


/** 
 * Class to hold all Talon FX/SRX configuration
 * Applicable to Phoenix 6
 *  */
public class TalonConfig extends BaseMotorConfig<TalonConfig> {

    /** 
     * Constructor
     * @param id - canbus ID
     * @param canbus - Name of canbus
     * @param name - name of motor for logging
     */
    public TalonConfig(int id, Canbus canbus, String name) {
        super(id, name, canbus);
        motorClass = MotorControllerType.TalonFX;
    }

    public TalonConfig(int id, String name, TalonConfig config) {
        super(id, name);
        copyBaseFields(config);
    }
}