// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.UtilsForChassis;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.UtilsVision.Camera;

import java.util.function.Supplier;

/** 
 * Calculates robot position changes accounting for vision latency.
 * This compensates for the delay between when vision data was captured
 * and when it's processed.
 */
public class CalculatePositionAndAngle {

    /**
     * Computes the change in robot position accounting for vision system latency.
     * 
     * @param camera The camera providing vision data
     * @param speeds Supplier of current chassis speeds (should be robot-relative)
     * @param getRobotAngle Supplier of current robot angle
     * @param dtseconds Time step in seconds (typically 0.02 for 50Hz loop)
     * @return Field-relative translation representing position change
     */
    public Translation2d computeDeltaPosition(
            Camera camera,
            Supplier<ChassisSpeeds> speeds, 
            Supplier<Rotation2d> getRobotAngle, 
            double dtseconds) {
        
        // Get network table for this camera
        NetworkTable table = NetworkTableInstance.getDefault().getTable(camera.getTableName());
        
        // Get latency in milliseconds and convert to seconds
        // tl = target latency (processing time)
        // cl = capture latency (network/camera delay)
        double targetLatencyMs = table.getEntry("tl").getDouble(0.0);
        double captureLatencyMs = table.getEntry("cl").getDouble(0.0);
        double latencyMs = targetLatencyMs + captureLatencyMs;
        double latencySeconds = latencyMs / 1000.0;
        
        // Total time period for position calculation
        double totalTime = dtseconds + latencySeconds;
        
        ChassisSpeeds currentSpeeds = speeds.get();
        Rotation2d robotAngle = getRobotAngle.get();
        
        // Calculate robot-relative movement during the total time period
        Translation2d robotRelativeMovement = new Translation2d(
                currentSpeeds.vxMetersPerSecond * totalTime,
                currentSpeeds.vyMetersPerSecond * totalTime
        );
        
        // Convert to field-relative by rotating by robot's current angle
        Translation2d fieldRelativeMovement = robotRelativeMovement.rotateBy(robotAngle);
        
        // Publish values to SmartDashboard for debugging
        String prefix = "Vision/" + camera.getTableName() + "/";
        
        // Latency information
        SmartDashboard.putNumber(prefix + "Target Latency (ms)", targetLatencyMs);
        SmartDashboard.putNumber(prefix + "Capture Latency (ms)", captureLatencyMs);
        SmartDashboard.putNumber(prefix + "Total Latency (ms)", latencyMs);
        SmartDashboard.putNumber(prefix + "Latency (s)", latencySeconds);
        
        // Time calculations
        SmartDashboard.putNumber(prefix + "DT (s)", dtseconds);
        SmartDashboard.putNumber(prefix + "Total Time (s)", totalTime);
        
        // Chassis speeds
        SmartDashboard.putNumber(prefix + "Vx (m/s)", currentSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber(prefix + "Vy (m/s)", currentSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber(prefix + "Omega (rad/s)", currentSpeeds.omegaRadiansPerSecond);
        
        // Robot angle
        SmartDashboard.putNumber(prefix + "Robot Angle (deg)", robotAngle.getDegrees());
        
        // Robot-relative movement
        SmartDashboard.putNumber(prefix + "Robot-Rel Delta X (m)", robotRelativeMovement.getX());
        SmartDashboard.putNumber(prefix + "Robot-Rel Delta Y (m)", robotRelativeMovement.getY());
        SmartDashboard.putNumber(prefix + "Robot-Rel Distance (m)", robotRelativeMovement.getNorm());
        
        // Field-relative movement (final output)
        SmartDashboard.putNumber(prefix + "Field-Rel Delta X (m)", fieldRelativeMovement.getX());
        SmartDashboard.putNumber(prefix + "Field-Rel Delta Y (m)", fieldRelativeMovement.getY());
        SmartDashboard.putNumber(prefix + "Field-Rel Distance (m)", fieldRelativeMovement.getNorm());
        
        return fieldRelativeMovement;
    }
}