package frc.demacia.utils.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.Sensors.PigeonConfig;

/**
 * Configuration class for swerve drive chassis.
 * 
 * <p>Contains all module configurations, physical dimensions, and motion constraints.</p>
 * 
 * <p><b>Example:</b></p>
 * <pre>
 * ChassisConfig config = new ChassisConfig(...)
 *     .withMaxLinearAccel(10.0)      // 10 m/s² max acceleration
 *     .withMaxOmegaVelocity(Math.toRadians(540))  // 540°/s rotation
 *     .withMaxRadius(0.4);           // 0.4m turn radius
 * </pre>
 */
public class ChassisConfig {
    public String name;

    public SwerveModuleConfig frontLeftModuleConfig;
    public SwerveModuleConfig frontRightModuleConfig;
    public SwerveModuleConfig backLeftModuleConfig;
    public SwerveModuleConfig backRightModuleConfig;

    public PigeonConfig pigeonConfig;

    public Translation2d frontLeftPosition;
    public Translation2d frontRightPosition;
    public Translation2d backLeftPosition;
    public Translation2d backRightPosition;

    public double cycleDt = 0.02;
    public double maxLinearAccel = 10;
    public double maxOmegaVelocity = Math.toRadians(540);
    public double maxRadialAccel = 6;
    public double maxRadius = 0.4;
    public double minOmegaDiff = Math.toRadians(20);
    public double maxDeltaVelocity = maxLinearAccel * cycleDt;
    public double maxVelocityToIgnoreRadius = maxRadius * maxOmegaVelocity;
    public double minVelocity = 1.5;

    public ChassisConfig(String name, SwerveModuleConfig frontLeftModuleConfig, SwerveModuleConfig frontRightModuleConfig, SwerveModuleConfig backLeftModuleConfig, SwerveModuleConfig backRightModuleConfig, PigeonConfig pigeonConfig, Translation2d frontLeftPosition, Translation2d frontRightPosition, Translation2d backLeftPosition, Translation2d backRightPosition){
        this.name = name;
        this.frontLeftModuleConfig = frontLeftModuleConfig;
        this.frontRightModuleConfig = frontRightModuleConfig;
        this.backLeftModuleConfig = backLeftModuleConfig;
        this.backRightModuleConfig = backRightModuleConfig;
        this.pigeonConfig = pigeonConfig;
        this.frontLeftPosition = frontLeftPosition;
        this.frontRightPosition = frontRightPosition;
        this.backLeftPosition = backLeftPosition;
        this.backRightPosition = backRightPosition;
    }

    /**
     * Sets the robot control loop period.
     * 
     * @param cycleDt Period in seconds (typically 0.02 for 50Hz)
     * @return this config for chaining
     */
    public ChassisConfig withCycleDt(double cycleDt){
        this.cycleDt = cycleDt;
        return this;
    }

    /**
     * Sets maximum linear acceleration.
     * 
     * <p>Higher values = more aggressive acceleration but risk of wheel slip.</p>
     * 
     * @param maxLinearAccel Maximum acceleration in m/s²
     * @return this config for chaining
     */
    public ChassisConfig withMaxLinearAccel(double maxLinearAccel){
        this.maxLinearAccel = maxLinearAccel;
        return this;
    }

    /**
     * Sets maximum rotational velocity.
     * 
     * @param maxOmegaVelocity Maximum rotation speed in rad/s
     * @return this config for chaining
     */
    public ChassisConfig withMaxOmegaVelocity(double maxOmegaVelocity){
        this.maxOmegaVelocity = maxOmegaVelocity;
        return this;
    }

    public ChassisConfig withMaxRadialAccel(double maxRadialAccel){
        this.maxRadialAccel = maxRadialAccel;
        return this;
    }

    /**
     * Sets maximum turning radius for smooth motion profiling.
     * 
     * <p>Smaller radius = tighter turns but requires slowing down.</p>
     * 
     * @param maxRadius Radius in meters
     * @return this config for chaining
     */
    public ChassisConfig withMaxRadius(double maxRadius){
        this.maxRadius = maxRadius;
        return this;
    }

    public ChassisConfig withMinOmegaDiff(double minOmegaDiff){
        this.minOmegaDiff = minOmegaDiff;
        return this;
    }

    public ChassisConfig withMaxDeltaVelocity(double maxDeltaVelocity){
        this.maxDeltaVelocity = maxDeltaVelocity;
        return this;
    }

    public ChassisConfig withMaxVelocityToIgnoreRadius(double maxVelocityToIgnoreRadius){
        this.maxVelocityToIgnoreRadius = maxVelocityToIgnoreRadius;
        return this;
    }

    /**
     * Sets minimum velocity during direction changes.
     * 
     * <p>Prevents the robot from stopping during sharp turns.</p>
     * 
     * @param minVelocity Minimum velocity in m/s
     * @return this config for chaining
     */
    public ChassisConfig withMinVelocity(double minVelocity){
        this.minVelocity = minVelocity;
        return this;
    }
}
