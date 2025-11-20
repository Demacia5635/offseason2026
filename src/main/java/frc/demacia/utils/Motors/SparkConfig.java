package frc.demacia.utils.Motors;

/** 
 * Class to hold all Spark motor configuration
 * Applicable to REV Spark Max/Flex
 *  */
public class SparkConfig extends BaseMotorConfig<SparkConfig> {

    // SparkMotorType motorType = SparkMotorType.SparkMax;

    /** 
     * Constructor
     * @param id - canbus ID
     * @param name - name of motor for logging
     */
    public SparkConfig(int id, String name) {
        super(id, name);
        motorClass = MotorControllerType.SparkMax;
    }

    public SparkConfig(int id, String name, SparkConfig config) {
        this(id,name);
        copyBaseFields(config);
    }
}