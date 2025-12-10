// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.chassis.Kinematics;

/** Add your docs here. */
public class KinematicsConstants {

    public record KinematicsConfig(double MAX_VELOCITY, double MAX_LINEAR_ACCEL, double MAX_RADIAL_ACCEL) {
    }

    public static final KinematicsConfig config = new KinematicsConfig(3.0, 10, 10.0);

    public static final double MAX_ALLOWED_MODULE_VELOCITY = 3;
    
    public static final double CYCLE_DT = 0.02;
    public static final double MAX_LINEAR_VELOCITY = 5;
    public static final double MAX_LINEAR_ACCEL = 10;
    public static final double MAX_OMEGA_ACCEL = Math.toRadians(720);
    public static final double MAX_OMEGA_VELOCITY = Math.toRadians(360);
    public static final double MAX_RADIAL_ACCEL = 10;
    public static final double MAX_SKID_ACCEL = 10;

}