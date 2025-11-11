// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testMotors.talonFX;

import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;
public final class Constants {
  public static final int ID = 10;
  public static final Canbus CAN_BUS = Canbus.Rio;
  public static final String NAME = "Test Motor";

  public static final double MAX_VOLT = 12;
  public static final double MAX_CURRENT = 40;
  public static final double RAMP_TIME = 0.3;

  /* the pid and ff constants of the motor */
  public static final double KP = 0;
  public static final double KI = 0;
  public static final double KD = 0;
  public static final double KS = 0;
  public static final double KV = 0;
  public static final double KA = 0;
  public static final double KG = 0;

  /* the motion magic constants of the motor */
  public static final double MOTION_MAGIC_VELOCITY = 0;
  public static final double MOTION_MAGIC_ACCELERATION = 0;
  public static final double MOTION_MAGIC_JERK = 0;
  public static final double MAX_POSITION_ERROR = 0.5;

  /* the channel of the limit switch of the arm angle motor */
  public static final int LIMIT_SWITCH_CHANNEL = 0;

  /* the basic configues of the motor */
  public static final boolean IS_BRAKE = true;
  public static final boolean IS_INVERTED = false;
  public static final double GEAR_RATIO = 1;

  /* The config of the motors based on the constants above */
  public static final TalonConfig CONFIG = new TalonConfig(ID, CAN_BUS, NAME)
    .withPID(KP, KI, KD, KS, KV, KA, KG)
    .withMotionParam(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
    .withBrake(IS_BRAKE)
    .withInvert(IS_INVERTED)
    .withRadiansMotor(GEAR_RATIO)
    .withRampTime(RAMP_TIME)
    .withVolts(MAX_VOLT)
    .withCurrent(MAX_CURRENT)
    .withMaxPositionError(MAX_POSITION_ERROR);
}
