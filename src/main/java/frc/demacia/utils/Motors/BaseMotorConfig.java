package frc.demacia.utils.Motors;

import com.ctre.phoenix6.CANBus;

/**
 * Abstract base class for motor configurations.
 * Contains common fields and methods shared between different motor controller types.
 * 
 * <p>This class provides a fluent API for configuring motors with method chaining.
 * All configuration methods return the config instance for easy chaining.</p>
 * 
 * <p><b>Supported Motor Types:</b></p>
 * <ul>
 *   <li>TalonFX - CTRE Talon FX (Falcon 500)</li>
 *   <li>TalonSRX - CTRE Talon SRX</li>
 *   <li>SparkMax - REV Spark Max</li>
 *   <li>SparkFlex - REV Spark Flex</li>
 * </ul>
 * 
 * <p><b>Example Usage:</b></p>
 * <pre>
 * TalonConfig config = new TalonConfig(1, Canbus.CANIvore, "ShooterMotor")
 *     .withVolts(12)
 *     .withCurrent(40)
 *     .withBrake(true)
 *     .withInvert(false)
 *     .withRadiansMotor(50.0)  // 50:1 gear ratio
 *     .withPID(0.1, 0, 0.01, 0.05, 0.12, 0, 0)
 *     .withMotionParam(100, 200, 400);
 * </pre>
 * 
 * @param <T> The concrete config type (for method chaining)
 * 
 * @see TalonConfig
 * @see SparkConfig
 * @see TalonSRXConfig
 */
public abstract class BaseMotorConfig<T extends BaseMotorConfig<T>> {
    
    /**
     * Enum representing available CAN bus types.
     * 
     * <p><b>Rio:</b> The default roboRIO CAN bus</p>
     * <p><b>CANIvore:</b> CTRE CANivore device for high-speed CAN communication</p>
     */
    public static enum Canbus { Rio("rio"), CANIvore("canivore");
    
        public CANBus canbus;
        private Canbus(String name) {
            this.canbus = new CANBus(name);
        }
    } 

    /**
     * Enum mapping motor controller types to their implementation classes.
     * Used internally to instantiate the correct motor class based on configuration.
     */
    public static enum MotorControllerType{TalonFX(TalonMotor.class), TalonSRX(TalonSRXMotor.class), SparkMax(SparkMotor.class), SparkFlex(SparkFlexMotor.class);
    
        Class<? extends MotorInterface> motorClass;
        private MotorControllerType(Class<? extends MotorInterface> motorClass) {
            this.motorClass = motorClass;
        }
    }

    public int id;                  // CAN bus ID
    public Canbus canbus = Canbus.Rio;
    public MotorControllerType motorClass  = MotorControllerType.TalonFX;
    public String name;             // Name of the motor - used for logging

    public double maxVolt = 12;     // Max Volt allowed
    public double minVolt = -12;    // Min Volts allowed
    public double maxCurrent = 40;  // Max current allowed
    public double rampUpTime = 0.3; // max power change time from 0 to full

    public boolean brake = true;    // brake/coast
    public double motorRatio = 1;   // motor to mechanism ratio
    public boolean inverted = false; // if to invert motor

    public double maxVelocity = 0;
    public double maxAcceleration = 0;
    public double maxJerk = 0;
    public double maxPositionError = 0.5;

    public CloseLoopParam[] pid = {new CloseLoopParam(), new CloseLoopParam(), new CloseLoopParam()};

    public boolean isMeterMotor = false;
    public boolean isDegreesMotor = false;
    public boolean isRadiansMotor = false;

    // enhanced ff
    public double kv2 = 0;
    public double kSin = 0;
    public double posToRad = 0;

    /**
     * Constructor for basic motor configuration.
     * 
     * @param id CAN bus ID of the motor controller (1-63)
     * @param name Descriptive name for logging and debugging (e.g., "LeftDrive", "Shooter")
     */
    public BaseMotorConfig(int id, String name) {
        this.id = id;
        this.name = name;
    }
    /**
     * Constructor with explicit CAN bus selection.
     * 
     * @param id CAN bus ID of the motor controller (1-63)
     * @param name Descriptive name for logging and debugging
     * @param canbus CAN bus type (Rio or CANIvore)
     */
    public BaseMotorConfig(int id, String name, Canbus canbus) {
        this(id,name);
        this.canbus = canbus;
    }
    
    public Class<? extends MotorInterface> getMotorClass() {
        return motorClass.motorClass;
    }

    /**
     * Sets the motor controller type.
     * 
     * @param motorClass The type of motor controller to use
     * @return this config for method chaining
     */
    @SuppressWarnings("unchecked")
    public T withMotorClass(MotorControllerType motorClass) {
        this.motorClass = motorClass;
        return (T) this;
    }

    /**
     * Sets voltage limits for the motor.
     * Automatically sets minVolt to negative of maxVolt.
     * 
     * <p><b>Typical values:</b></p>
     * <ul>
     *   <li>12V - Standard FRC battery voltage</li>
     *   <li>10V - Reduced voltage for testing</li>
     * </ul>
     * 
     * @param maxVolt Maximum voltage (typically 12V for FRC)
     * @return this config for method chaining
     */
    @SuppressWarnings("unchecked")
    public T withVolts(double maxVolt) {
        this.maxVolt = maxVolt;
        this.minVolt = -maxVolt;
        return (T) this;
    }

    /**
     * Sets the neutral mode (brake vs coast).
     * 
     * <p><b>Brake mode:</b> Motor actively resists movement when stopped (high holding torque)</p>
     * <p><b>Coast mode:</b> Motor freewheels when stopped (low friction)</p>
     * 
     * @param brake true for brake mode, false for coast mode
     * @return this config for method chaining
     */
    @SuppressWarnings("unchecked")
    public T withBrake(boolean brake) {
        this.brake = brake;
        return (T) this;
    }

    /**
     * Sets motor direction inversion.
     * 
     * <p>Use this to ensure positive output results in the desired direction.
     * Recommended: Set inversion in config rather than negating values in code.</p>
     * 
     * @param invert true to invert motor direction, false for normal direction
     * @return this config for method chaining
     */
    @SuppressWarnings("unchecked")
    public T withInvert(boolean invert) {
        this.inverted = invert;
        return (T) this;
    }

    /**
     * Sets the ramp rate for motor output changes.
     * 
     * <p>Limits the rate of change of motor output to prevent mechanical stress,
     * brownouts, and tipping. Time is measured from 0 to full power.</p>
     * 
     * <p><b>Typical values:</b></p>
     * <ul>
     *   <li>0.1-0.3s - Drive motors (balance responsiveness vs smoothness)</li>
     *   <li>0.05s - Mechanisms requiring quick response</li>
     *   <li>0 - No ramping (instant response)</li>
     * </ul>
     * 
     * @param rampTime Time in seconds to ramp from 0 to full power
     * @return this config for method chaining
     */
    @SuppressWarnings("unchecked")
    public T withRampTime(double rampTime) {
        this.rampUpTime = rampTime;
        return (T) this;
    }

    /**
     * Configures motor for linear motion (meters).
     * Automatically calculates the conversion factor from motor rotations to linear distance.
     * 
     * <p><b>Formula:</b> motorRatio = gearRatio / (diameter * π)</p>
     * 
     * <p><b>Example:</b></p>
     * <pre>
     * // 10:1 gearbox with 4-inch (0.1016m) diameter wheels
     * config.withMeterMotor(10.0, 0.1016);
     * 
     * // Now setMotion(1.0) will move 1 meter
     * </pre>
     * 
     * @param gearRatio Gear ratio from motor shaft to wheel/mechanism (output/motor)
     * @param diameter Wheel or pulley diameter in meters
     * @return this config for method chaining
     */
    @SuppressWarnings("unchecked")
    public T withMeterMotor(double gearRatio, double diameter) {
        this.motorRatio = gearRatio / (diameter * Math.PI);
        isMeterMotor = true;
        isRadiansMotor = false;
        isDegreesMotor = false;
        return (T) this;
    }

    /**
     * Configures motor for rotational motion (radians).
     * Automatically calculates the conversion factor from motor rotations to radians.
     * 
     * <p><b>Formula:</b> motorRatio = gearRatio / (2π)</p>
     * 
     * <p><b>Example:</b></p>
     * <pre>
     * // 50:1 gearbox on an arm
     * config.withRadiansMotor(50.0);
     * 
     * // Now setAngle(Math.PI/2) will rotate to 90 degrees
     * </pre>
     * 
     * @param gearRatio Gear ratio from motor shaft to mechanism (output/motor)
     * @return this config for method chaining
     */
    @SuppressWarnings("unchecked")
    public T withRadiansMotor(double gearRatio) {
        this.motorRatio = gearRatio / (Math.PI * 2);
        isMeterMotor = false;
        isRadiansMotor = true;
        isDegreesMotor = false;
        return (T) this;
    }

    /**
     * Configures motor for rotational motion (degrees).
     * Automatically calculates the conversion factor from motor rotations to degrees.
     * 
     * <p><b>Formula:</b> motorRatio = gearRatio / 360</p>
     * 
     * <p><b>Example:</b></p>
     * <pre>
     * // 100:1 gearbox on a turret
     * config.withDegreesMotor(100.0);
     * 
     * // Now setAngle(90) will rotate to 90 degrees
     * </pre>
     * 
     * @param gearRatio Gear ratio from motor shaft to mechanism (output/motor)
     * @return this config for method chaining
     */
    @SuppressWarnings("unchecked")
    public T withDegreesMotor(double gearRatio) {
        this.motorRatio = gearRatio / 360;
        isMeterMotor = false;
        isRadiansMotor = false;
        isDegreesMotor = true;
        return (T) this;
    }

    /**
     * Sets the maximum acceptable position error for closed-loop control.
     * 
     * <p>Used to determine when a mechanism has "arrived" at its target position.
     * Units depend on motor configuration (meters, radians, or degrees).</p>
     * 
     * @param maxPositionError Maximum acceptable error in configured units
     * @return this config for method chaining
     */
    @SuppressWarnings("unchecked")
    public T withMaxPositionError(double maxPositionError) {
        this.maxPositionError = maxPositionError;
        return (T) this;
    }

    /**
     * Sets the current limit for the motor.
     * 
     * <p>Protects motors from damage and prevents brownouts by limiting current draw.
     * The exact behavior varies by motor controller type.</p>
     * 
     * <p><b>Typical values:</b></p>
     * <ul>
     *   <li>30-40A - Standard limit for most mechanisms</li>
     *   <li>60-80A - High-power applications (drive motors)</li>
     *   <li>20A - Low-power mechanisms (intakes, small arms)</li>
     * </ul>
     * 
     * @param maxCurrent Maximum current in amps
     * @return this config for method chaining
     */
    @SuppressWarnings("unchecked")
    public T withCurrent(double maxCurrent) {
        this.maxCurrent = maxCurrent;
        return (T) this;
    }

    /**
     * Configures motion profiling parameters for Motion Magic / MAXMotion control.
     * 
     * <p>These parameters define the acceleration profile for smooth, controlled motion.
     * Units are in configured motor units (meters/sec, radians/sec, or rotations/sec).</p>
     * 
     * <p><b>Example:</b></p>
     * <pre>
     * // For a fast-moving arm
     * config.withMotionParam(
     *     5.0,    // 5 rad/s max velocity
     *     10.0,   // 10 rad/s² acceleration
     *     50.0    // 50 rad/s³ jerk
     * );
     * </pre>
     * 
     * @param maxVelocity Maximum velocity in configured units per second
     * @param maxAcceleration Maximum acceleration in configured units per second²
     * @param maxJerk Maximum jerk (rate of acceleration change) in configured units per second³
     * @return this config for method chaining
     */
    @SuppressWarnings("unchecked")
    public T withMotionParam(double maxVelocity, double maxAcceleration, double maxJerk) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxJerk = maxJerk;
        return (T) this;
    }

    /**
     * Sets enhanced feed-forward parameters for advanced control.
     * 
     * <p>These parameters enable compensation for non-linear dynamics:</p>
     * <ul>
     *   <li><b>kv2:</b> Velocity-squared term for air resistance/friction (rare)</li>
     *   <li><b>kSin:</b> Sine term for gravitational compensation (arms, elevators)</li>
     *   <li><b>posToRad:</b> Conversion factor from position to radians for sine term</li>
     * </ul>
     * 
     * <p><b>Example:</b> Arm with gravity compensation</p>
     * <pre>
     * config.withFeedForward(
     *     0,           // No velocity-squared term
     *     0.5,         // Gravity compensation strength
     *     Math.PI/90   // Convert position (degrees) to radians
     * );
     * </pre>
     * 
     * @param kv2 Velocity-squared coefficient
     * @param ksin Sine (gravity) coefficient in volts
     * @param posToRad Position to radians conversion factor
     * @return this config for method chaining
     */
    @SuppressWarnings("unchecked")
    public T withFeedForward(double kv2, double ksin, double posToRad) {
        this.kv2 = kv2;
        this.kSin = ksin;
        this.posToRad = posToRad;
        return (T)this;
    }

    /**
     * Sets PID and feed-forward gains for slot 0.
     * 
     * <p><b>Parameters explained:</b></p>
     * <ul>
     *   <li><b>kp:</b> Proportional gain - corrects current error</li>
     *   <li><b>ki:</b> Integral gain - eliminates steady-state error (use sparingly)</li>
     *   <li><b>kd:</b> Derivative gain - dampens oscillations</li>
     *   <li><b>ks:</b> Static friction compensation in volts</li>
     *   <li><b>kv:</b> Velocity feed-forward in volts/(unit/sec)</li>
     *   <li><b>ka:</b> Acceleration feed-forward in volts/(unit/sec²)</li>
     *   <li><b>kg:</b> Gravity compensation in volts (constant)</li>
     * </ul>
     * 
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param ks Static friction feed-forward (volts)
     * @param kv Velocity feed-forward (volts per unit/sec)
     * @param ka Acceleration feed-forward (volts per unit/sec²)
     * @param kg Gravity feed-forward (volts)
     * @return this config for method chaining
     */
    public T withPID(double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        return (T)withPID(0, kp, ki, kd, ks, kv, ka, kg);
    }
    
    /**
     * Sets PID and feed-forward gains for a specific slot (0-2).
     * 
     * <p>Multiple slots allow switching between different control strategies on-the-fly.
     * For example: slot 0 for position control, slot 1 for velocity control.</p>
     * 
     * @param slot Slot number (0-2)
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param ks Static friction feed-forward (volts)
     * @param kv Velocity feed-forward (volts per unit/sec)
     * @param ka Acceleration feed-forward (volts per unit/sec²)
     * @param kg Gravity feed-forward (volts)
     * @return this config for method chaining
     */
    @SuppressWarnings("unchecked")
    public T withPID(int slot, double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        pid[slot] = new CloseLoopParam(kp, ki, kd, ks, kv, ka, kg);
        return (T)this;
    }

    /**
     * Sets the CAN bus for this motor controller.
     * 
     * @param canbus CAN bus type (Rio or CANIvore)
     * @return this config for method chaining
     */
    @SuppressWarnings("unchecked")
    public T withCanbus(Canbus canbus) {
        this.canbus = canbus;
        return (T)this;
    }

    /**
     * Copies all configuration fields from another BaseMotorConfig.
     * 
     * <p>Useful for creating multiple motors with similar configurations:</p>
     * <pre>
     * TalonConfig leftConfig = new TalonConfig(1, Canbus.CANIvore, "LeftDrive")
     *     .withVolts(12)
     *     .withCurrent(40)
     *     .withPID(...);
     * 
     * // Right motor inherits all settings, just change ID and inversion
     * TalonConfig rightConfig = new TalonConfig(2, "RightDrive", leftConfig)
     *     .withInvert(true);
     * </pre>
     * 
     * @param other The config to copy from
     */
    protected void copyBaseFields(BaseMotorConfig<?> other) {
        this.canbus = other.canbus;
        this.maxVolt = other.maxVolt;
        this.minVolt = other.minVolt;
        this.maxCurrent = other.maxCurrent;
        this.rampUpTime = other.rampUpTime;
        this.brake = other.brake;
        this.motorRatio = other.motorRatio;
        this.inverted = other.inverted;
        this.maxCurrent = other.maxCurrent;
        this.kv2 = other.kv2;
        this.kSin = other.kSin;
        this.posToRad = other.posToRad;
        this.maxAcceleration = other.maxAcceleration;
        this.maxVelocity = other.maxVelocity;
        this.maxJerk = other.maxJerk;
        this.pid[0].set(other.pid[0]);
        this.pid[1].set(other.pid[1]);
        this.pid[2].set(other.pid[2]);
        this.maxPositionError = other.maxPositionError;
        this.isDegreesMotor = other.isDegreesMotor;
        this.isMeterMotor = other.isMeterMotor;
        this.isRadiansMotor = other.isRadiansMotor;
   }
}