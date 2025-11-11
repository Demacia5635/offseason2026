package frc.demacia.utils.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.Sensors.PigeonConfig;

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

    public ChassisConfig withCycleDt(double cycleDt){
        this.cycleDt = cycleDt;
        return this;
    }

    public ChassisConfig withMaxLinearAccel(double maxLinearAccel){
        this.maxLinearAccel = maxLinearAccel;
        return this;
    }

    public ChassisConfig withMaxOmegaVelocity(double maxOmegaVelocity){
        this.maxOmegaVelocity = maxOmegaVelocity;
        return this;
    }

    public ChassisConfig withMaxRadialAccel(double maxRadialAccel){
        this.maxRadialAccel = maxRadialAccel;
        return this;
    }

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

    public ChassisConfig withMinVelocity(double minVelocity){
        this.minVelocity = minVelocity;
        return this;
    }
}
