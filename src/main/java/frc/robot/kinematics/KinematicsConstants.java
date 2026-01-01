// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kinematics;

/** Add your docs here. */
public class KinematicsConstants {

    public record KinematicsConfig(double MAX_VELOCITY, double MAX_FRONT_ACCEL, double MAX_SIDE_ACCEL, double MAX_SKID_ACCEL) {
    }

    public static final KinematicsConfig config = new KinematicsConfig(4,  10, 8, 30);

    public static final double MAX_ALLOWED_MODULE_VELOCITY = 4;
    public static final double CYCLE_DT = 0.02;
    //public static final double MAX_DELTA_V = 10 * CYCLE_DT;
    // public static final double MAX_FORWARD_ACCEL = 10;

}
