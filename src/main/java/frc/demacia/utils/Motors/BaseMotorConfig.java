package frc.demacia.utils.Motors;

import com.ctre.phoenix6.CANBus;

/**
 * Abstract base class for motor configurations
 * Contains common fields and methods shared between different motor controller types
 */
public abstract class BaseMotorConfig<T extends BaseMotorConfig<T>> {
    
    public static enum Canbus { Rio("rio"), CANIvore("canivore");
    
        public CANBus canbus;
        private Canbus(String name) {
            this.canbus = new CANBus(name);
        }
    } 

    public static enum MotorControllerType{TalonFX(TalonMotor.class), TalonSRX(TalonSRXMotor.class), SparkMax(SparkMotor.class), SparkFlex(SparkFlexMotor.class);
    
        Class<? extends MotorInterface> motorClass;
        private MotorControllerType(Class<? extends MotorInterface> motorClass) {
            this.motorClass = motorClass;
        }
    }

    // public static enum MotorControllerType {TalonFX, SparkMax, SparkFlex, TalonSRX};

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
     * Constructor
     * @param id - CAN bus ID
     * @param name - name of motor for logging
     */
    public BaseMotorConfig(int id, String name) {
        this.id = id;
        this.name = name;
    }
    public BaseMotorConfig(int id, String name, Canbus canbus) {
        this(id,name);
        this.canbus = canbus;
    }
    
    public Class<? extends MotorInterface> getMotorClass() {
        return motorClass.motorClass;
    }

    @SuppressWarnings("unchecked")
    public T withMotorClass(MotorControllerType motorClass) {
        this.motorClass = motorClass;
        return (T) this;
    }

    /**
     * Set voltage limits
     * @param maxVolt maximum voltage
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public T withVolts(double maxVolt) {
        this.maxVolt = maxVolt;
        this.minVolt = -maxVolt;
        return (T) this;
    }

    /**
     * Set brake mode
     * @param brake true for brake mode, false for coast
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public T withBrake(boolean brake) {
        this.brake = brake;
        return (T) this;
    }

    /**
     * Set motor inversion
     * @param invert true to invert motor direction
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public T withInvert(boolean invert) {
        this.inverted = invert;
        return (T) this;
    }

    /**
     * Set ramp time
     * @param rampTime time in seconds to ramp from 0 to full power
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public T withRampTime(double rampTime) {
        this.rampUpTime = rampTime;
        return (T) this;
    }

    /**
     * Configure motor for linear motion (meters)
     * @param gearRatio gear ratio from motor to mechanism
     * @param circumference wheel/pulley circumference in meters
     * @return this config for chaining
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
     * Configure motor for rotational motion (radians)
     * @param gearRatio gear ratio from motor to mechanism
     * @return this config for chaining
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
     * Configure motor for rotational motion (degrees)
     * @param gearRatio gear ratio from motor to mechanism
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public T withDegreesMotor(double gearRatio) {
        this.motorRatio = gearRatio / 360;
        isMeterMotor = false;
        isRadiansMotor = false;
        isDegreesMotor = true;
        return (T) this;
    }

    
    @SuppressWarnings("unchecked")
    public T withMaxPositionError(double maxPositionError) {
        this.maxPositionError = maxPositionError;
        return (T) this;
    }

    /**
     * Set current limit - implementation varies by motor controller
     * @param maxCurrent maximum current in amps
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public T withCurrent(double maxCurrent) {
        this.maxCurrent = maxCurrent;
        return (T) this;
    }

    /**
     * Set max velocity, accelration and jerk for magic or max motion
     * @param maxCurrent maximum current in amps
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public T withMotionParam(double maxVelocity, double maxAcceleration, double maxJerk) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxJerk = maxJerk;
        return (T) this;
    }

    /** 
     * Set enhanced feed forward prams
     * @param kv2
     * @param ksin
     * @param posToRad
     * @return TalonConfig
     */
    @SuppressWarnings("unchecked")
    public T withFeedForward(double kv2, double ksin, double posToRad) {
        this.kv2 = kv2;
        this.kSin = ksin;
        this.posToRad = posToRad;
        return (T)this;
    }

    /** 
     * Set pid
     * @param kp
     * @param ki
     * @param kd
     * @param ks
     * @param kv
     * @return TalonConfig
     */
    public T withPID(double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        return (T)withPID(0, kp, ki, kd, ks, kv, ka, kg);
    }
    /** 
     * Set pid
     * @param slot
     * @param kp
     * @param ki
     * @param kd
     * @param ks
     * @param kv
     * @param ka
     * @param kg
     * @return TalonConfig
     */
    @SuppressWarnings("unchecked")
    public T withPID(int slot, double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        pid[slot] = new CloseLoopParam(kp, ki, kd, ks, kv, ka, kg);
        return (T)this;
    }

    @SuppressWarnings("unchecked")
    public T withCanbus(Canbus canbus) {
        this.canbus = canbus;
        return (T)this;
    }

    /**
     * Copy common fields from another BaseMotorConfig
     * @param other the config to copy from
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

/* Alphabetic order:
 * BaseMotorConfig(int id, String name)
 * BaseMotorConfig(int id, String name, Canbus canbus)
 * withVolts(double maxVolt)
 * withBrake(boolean brake)
 * withInvert(boolean invert)
 * withRampTime(double rampTime)
 * withMeterMotor(double gearRatio, double diameter)
 * withRadiansMotor(double gearRatio)
 * withDegreesMotor(double gearRatio)
 * withMaxPositionError(double maxPositionError)
 * withCurrent(double maxCurrent)
 * withMotionParam(double maxVelocity, double maxAcceleration, double maxJerk)
 * withFeedForward(double kv2, double ksin, double posToRad)
 * withPID(double kp, double ki, double kd, double ks, double kv, double ka, double kg)
 * withCanbus(Canbus canbus)
 * copyBaseFields(BaseMotorConfig<?> other)
 */