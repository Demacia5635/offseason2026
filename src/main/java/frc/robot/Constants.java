// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.instructions.step1;

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


  public static class motorConstants {
        /* all the main configs of the motor */
        public static final int ID = step1.id;
        public static final Canbus CAN_BUS = step1.canbus;
        public static final String NAME = "Test Motor";
        public static final double MAX_POWER = step1.maxRobotPower;
        public static final double MIN_POWER = step1.minRobotPower;
        public static final double TIME =step1.TIME;
        public static final double JUMP = step1.JUMP;
        
        public static final double GEAR_RATIO = step1.gear_ratio;

        /* The config of the motors based on the constants above */
        public static final TalonConfig CONFIG = new TalonConfig(ID, CAN_BUS, NAME)
                .withRadiansMotor(GEAR_RATIO);
    }
}