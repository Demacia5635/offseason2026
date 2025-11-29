package frc.demacia.utils.Sensors;

import com.ctre.phoenix6.CANBus;

import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;

/**
 * Configuration for CTRE Pigeon2 IMU.
 * 
 * <p><b>Mounting Offsets:</b> Use when Pigeon is not mounted perfectly aligned
 * with robot axes. Set offsets to correct for mounting angle.</p>
 * 
 * <p><b>Scalars:</b> Typically left at 1.0 unless compensating for known error.</p>
 * 
 * <p><b>Example:</b></p>
 * <pre>
 * PigeonConfig config = new PigeonConfig(13, CANBus.CANivore, "Gyro")
 *     .withYawOffset(0)        // Mounted aligned
 *     .withInvert(false)       // Normal direction
 *     .withTemperatureCompensation(true);  // Enable temp compensation
 * </pre>
 */
public class PigeonConfig extends BaseSensorConfig<PigeonConfig>{
    public double pitchOffset = 0;
    public double rollOffset = 0;
    public double yawOffset = 0;
    public double xScalar = 1;
    public double yScalar = 1;
    public double zScalar = 1;
    public boolean compass = false;
    public boolean temperatureCompensation = false;
    public boolean noMotionCalibration = false;
    
    /**
     * Creates Pigeon2 configuration.
     * 
     * @param id CAN bus ID
     * @param canbus CANBus instance
     * @param name Descriptive name for logging
     */
    public PigeonConfig(int id, Canbus canbus, String name) {
        super(id, canbus, name);
        sensorType = Pigeon.class;
    }

    /**
     * Sets pitch mounting offset.
     * 
     * @param pitchOffset Offset in degrees
     * @return this config for method chaining
     */
    public PigeonConfig withPitchOffset(double pitchOffset) {
        this.pitchOffset = pitchOffset;
        return this;
    }

    /**
     * Sets roll mounting offset.
     * 
     * @param rollOffset Offset in degrees
     * @return this config for method chaining
     */
    public PigeonConfig withRollOffset(double rollOffset) {
        this.rollOffset = rollOffset;
        return this;
    }

    /**
     * Sets yaw mounting offset.
     * 
     * @param yawOffset Offset in degrees
     * @return this config for method chaining
     */
    public PigeonConfig withYawOffset(double yawOffset) {
        this.yawOffset = yawOffset;
        return this;
    }

    /**
     * Sets X-axis scaling factor.
     * 
     * @param xScalar Scaling multiplier (typically 1.0)
     * @return this config for method chaining
     */
    public PigeonConfig withXScalar(double xScalar) {
        this.xScalar = xScalar;
        return this;
    }

    /**
     * Sets Y-axis scaling factor.
     * 
     * @param yScalar Scaling multiplier (typically 1.0)
     * @return this config for method chaining
     */
    public PigeonConfig withYScalar(double yScalar) {
        this.yScalar = yScalar;
        return this;
    }

    /**
     * Sets Z-axis scaling factor.
     * 
     * @param zScalar Scaling multiplier (typically 1.0)
     * @return this config for method chaining
     */
    public PigeonConfig withZScalar(double zScalar) {
        this.zScalar = zScalar;
        return this;
    }

    /**
     * Enables/disables compass functionality.
     * 
     * <p>Compass uses magnetometer for absolute heading but can be affected
     * by metal and magnets on robot.</p>
     * 
     * @param compass true to enable compass
     * @return this config for method chaining
     */
    public PigeonConfig withCompass(boolean compass) {
        this.compass = compass;
        return this;
    }

    /**
     * Enables/disables temperature compensation.
     * 
     * <p>Compensates for gyro drift due to temperature changes.
     * Recommended to enable for competition.</p>
     * 
     * @param temperatureCompensation true to enable
     * @return this config for method chaining
     */
    public PigeonConfig withTemperatureCompensation(boolean temperatureCompensation) {
        this.temperatureCompensation = temperatureCompensation;
        return this;
    }

    /**
     * Enables/disables no-motion calibration.
     * 
     * <p>Automatically calibrates gyro when robot is stationary.
     * Usually beneficial to enable.</p>
     * 
     * @param noMotionCalibration true to enable
     * @return this config for method chaining
     */
    public PigeonConfig withNoMotionCalibration(boolean noMotionCalibration) {
        this.noMotionCalibration = noMotionCalibration;
        return this;
    }
}
