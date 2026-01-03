// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.UtilsForChassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.UtilsVision.Camera;
import java.util.function.Supplier;

/**
 * Calculates robot position changes accounting for vision latency.
 * This compensates for the delay between when vision data was captured
 * and when it's processed by extrapolating the robot's movement during
 * the latency period.
 */
public class CalculatePositionAndAngle {
    
    public static Pose2d computeFuturePosition(ChassisSpeeds speeds, Pose2d currentPose, double dtSeconds) {
        Pose2d poseAtTime = new Pose2d(
            currentPose.getX() + (speeds.vxMetersPerSecond * dtSeconds),
            currentPose.getY() + (speeds.vyMetersPerSecond * dtSeconds),
            currentPose.getRotation().plus(new Rotation2d((speeds.omegaRadiansPerSecond * dtSeconds))));
        return poseAtTime;
    }
}