// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final double MAX_VELOCITY = 0;  
  public static final double MAX_ACCELERATION = 0;
  public static final double MAX_JERK = 0;

  public static final double kp = 0.1;
  public static final double KI = 0;
  public static final double KD = 0;
  public static final double ks = 0; 
  public static final double kv = 0;
  public static final double ka = 0;
  public static final double kg = 0;

  public static final TalonConfig config = new TalonConfig(20, Canbus.Rio, "engle cange")
    .withBrake(true)
  .withRadiansMotor(64d)
  .withPID(kp, KI, KD, ks, kv, ka, kg)
  .withMotionParam(MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK); 
}