// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testChassis;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.Sensors.CancoderConfig;
import frc.demacia.utils.Sensors.PigeonConfig;
import frc.demacia.utils.chassis.ChassisConfig;
import frc.demacia.utils.chassis.SwerveModuleConfig;

public final class Constants {
  public static final String NAME = "test chassis";
  public static final double STEER_GEAR_RATIO = 151.0/7.0;
    public static final double DRIVE_GEAR_RATIO = 8.14;
    
    public static final double STEER_KP = 4.1;
    public static final double STEER_KI = 0.9;
    public static final double STEER_KD = 0;
    public static final double STEER_KS = 0.19817640545050964;
    public static final double STEER_KV = 0.3866402641515461;
    public static final double STEER_KA = 0.05;

    public static final double DRIVE_KP = 19;
    public static final double DRIVE_KI = 0;
    public static final double DRIVE_KD = 0;
    public static final double DRIVE_KS = 0.14677232883614777;
    public static final double DRIVE_KV = 2.947;
    public static final double DRIVE_KA = 0.08058;

    public static final double MOTION_MAGIC_VEL = 15 * 2 * Math.PI;
    public static final double MOTION_MAGIC_ACCEL = 8 * 2 * Math.PI;
    public static final double MOTION_MAGIC_JERK = 160 * 2 * Math.PI;

    public static final double RAMP_TIME_STEER = 0.25;
    public static final double wheelDiameter = 0.1;

  public static final SwerveModuleConfig frontLeftModuleConfig = new SwerveModuleConfig(
    NAME + " frontLeft",
    new TalonConfig(2, Canbus.Rio, "frontLeft steer")
    .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, STEER_KA, 0)
    .withMotionParam(MOTION_MAGIC_VEL, MOTION_MAGIC_ACCEL, MOTION_MAGIC_JERK)
    .withBrake(true)
    .withInvert(true)
    .withRadiansMotor(STEER_GEAR_RATIO)
    .withRampTime(RAMP_TIME_STEER),
    new TalonConfig(1, Canbus.Rio, "frontLeft drive")
    .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, 0)
    .withBrake(true)
    .withInvert(true)
    .withMeterMotor(DRIVE_GEAR_RATIO, wheelDiameter),
    new CancoderConfig(3, new CANBus("rio"), "frontLeft cancoder"))
    .withSteerOffset(0.456055*2*Math.PI);
  public static final SwerveModuleConfig frontRightModuleConfig = new SwerveModuleConfig(
    NAME + " frontRight",
    new TalonConfig(5, Canbus.Rio, "frontRight steer")
    .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, STEER_KA, 0)
    .withMotionParam(MOTION_MAGIC_VEL, MOTION_MAGIC_ACCEL, MOTION_MAGIC_JERK)
    .withBrake(true)
    .withInvert(true)
    .withRadiansMotor(STEER_GEAR_RATIO)
    .withRampTime(RAMP_TIME_STEER),
    new TalonConfig(4, Canbus.Rio, "frontRight drive")
    .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, 0)
    .withBrake(true)
    .withInvert(true)
    .withMeterMotor(DRIVE_GEAR_RATIO, wheelDiameter),
    new CancoderConfig(6, new CANBus("rio"), "frontRight cancoder"))
    .withSteerOffset(0.53 * 2 * Math.PI);
  public static final SwerveModuleConfig backLeftModuleConfig = new SwerveModuleConfig(
    NAME + " backLeft",
    new TalonConfig(11, Canbus.Rio, "backLeft steer")
    .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, STEER_KA, 0)
    .withMotionParam(MOTION_MAGIC_VEL, MOTION_MAGIC_ACCEL, MOTION_MAGIC_JERK)
    .withBrake(true)
    .withInvert(true)
    .withRadiansMotor(STEER_GEAR_RATIO)
    .withRampTime(RAMP_TIME_STEER),
    new TalonConfig(10, Canbus.Rio, "backLeft drive")
    .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, 0)
    .withBrake(true)
    .withInvert(true)
    .withMeterMotor(DRIVE_GEAR_RATIO, wheelDiameter),
    new CancoderConfig(9, new CANBus("rio"), "backLeft cancoder"))
    .withSteerOffset(1.754883*2*Math.PI);
  public static final SwerveModuleConfig backRightModuleConfig = new SwerveModuleConfig(
    NAME + " backRight",
    new TalonConfig(8, Canbus.Rio, "backRight steer")
    .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, STEER_KA, 0)
    .withMotionParam(MOTION_MAGIC_VEL, MOTION_MAGIC_ACCEL, MOTION_MAGIC_JERK)
    .withBrake(true)
    .withInvert(true)
    .withRadiansMotor(STEER_GEAR_RATIO)
    .withRampTime(RAMP_TIME_STEER),
    new TalonConfig(7, Canbus.Rio, "backRight drive")
    .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, 0)
    .withBrake(true)
    .withInvert(true)
    .withMeterMotor(DRIVE_GEAR_RATIO, wheelDiameter),
    new CancoderConfig(12, new CANBus("rio"), "backRight cancoder"))
    .withSteerOffset(1.233643*2*Math.PI);
  public static final CANBus PIGEO_CANBUS = new CANBus("rio");
  public static final PigeonConfig pigeonConfig = new PigeonConfig(14, PIGEO_CANBUS, NAME + " pigeon");
  public static final Translation2d frontLeftPosition = new Translation2d(0.34, 0.29);
  public static final Translation2d frontRightPosition = new Translation2d(0.34, -0.29);
  public static final Translation2d backLeftPosition = new Translation2d(-0.34, 0.29);
  public static final Translation2d backRightPosition = new Translation2d(-0.34, -0.29);

  public static final ChassisConfig CHASSIS_CONFIG = new ChassisConfig(
    NAME,
    frontLeftModuleConfig,
    frontRightModuleConfig,
    backLeftModuleConfig,
    backRightModuleConfig,
    pigeonConfig,
    frontLeftPosition,
    frontRightPosition,
    backLeftPosition,
    backRightPosition);
}
