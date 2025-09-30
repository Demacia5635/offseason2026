// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.UtilsForChassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.UtilsVision.Camera;

import java.util.function.Supplier;

/**
 * Test harness for CalculatePositionAndAngle.
 * Creates a fake camera and simulates vision latency from SmartDashboard inputs.
 */
public class TestCalculatePositionAndAngle {
    
    private static Camera testCamera;
    private static CalculatePositionAndAngle calculator;
    
    /**
     * Initialize the test with a fake camera.
     * Call this once in Robot.robotInit()
     */
    public static void initializeTest() {
        // Create a test camera (the physical properties don't matter for this test)
        testCamera = new Camera(
            "test",                              // Name (will create table "limelight-test")
            new Translation2d(0, 0),            // Robot to camera position
            0.5,                                 // Camera height
            0.0,                                 // Pitch
            0.0                                  // Yaw
        );
        
        // Create the calculator instance
        calculator = new CalculatePositionAndAngle();
        
        // Set up default test values
        SmartDashboard.putNumber("Test/Input/Target Latency (ms)", 11.0);
        SmartDashboard.putNumber("Test/Input/Capture Latency (ms)", 20.0);
        SmartDashboard.putNumber("Test/Input/DT (seconds)", 0.02);
        SmartDashboard.putNumber("Test/Input/Vx (m/s)", 1.0);
        SmartDashboard.putNumber("Test/Input/Vy (m/s)", 0.5);
        SmartDashboard.putNumber("Test/Input/Omega (rad/s)", 0.0);
        SmartDashboard.putNumber("Test/Input/Robot Angle (deg)", 45.0);
        
        SmartDashboard.putString("Test/Instructions", 
            "Change 'Test/Input' values. Results appear in 'Test/Output'");
    }
    
    /**
     * Run the test continuously.
     * Call this in Robot.robotPeriodic()
     */
    public static void runTest() {
        if (testCamera == null || calculator == null) {
            SmartDashboard.putString("Test/Error", "Call initializeTest() first!");
            return;
        }
        
        // ========== READ INPUTS FROM SMARTDASHBOARD ==========
        
        double targetLatencyMs = SmartDashboard.getNumber("Test/Input/Target Latency (ms)", 11.0);
        double captureLatencyMs = SmartDashboard.getNumber("Test/Input/Capture Latency (ms)", 20.0);
        double dtseconds = SmartDashboard.getNumber("Test/Input/DT (seconds)", 0.02);
        double vxMetersPerSec = SmartDashboard.getNumber("Test/Input/Vx (m/s)", 1.0);
        double vyMetersPerSec = SmartDashboard.getNumber("Test/Input/Vy (m/s)", 0.5);
        double omegaRadPerSec = SmartDashboard.getNumber("Test/Input/Omega (rad/s)", 0.0);
        double robotAngleDeg = SmartDashboard.getNumber("Test/Input/Robot Angle (deg)", 45.0);
        
        // ========== WRITE LATENCY TO NETWORKTABLES (SIMULATE CAMERA) ==========
        
        // Write to the NetworkTable that your Camera points to
        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(testCamera.getTableName());
        limelightTable.getEntry("tl").setDouble(targetLatencyMs);
        limelightTable.getEntry("cl").setDouble(captureLatencyMs);
        
        // ========== CREATE SUPPLIERS FOR THE FUNCTION ==========
        
        final ChassisSpeeds speeds = new ChassisSpeeds(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec);
        Supplier<ChassisSpeeds> speedSupplier = () -> speeds;
        
        final Rotation2d robotAngle = Rotation2d.fromDegrees(robotAngleDeg);
        Supplier<Rotation2d> angleSupplier = () -> robotAngle;
        
        // ========== CALL YOUR ACTUAL FUNCTION ==========
        
        Translation2d result = calculator.computeDeltaPosition(
            testCamera,
            speedSupplier,
            angleSupplier,
            dtseconds
        );
        
        // ========== CALCULATE INTERMEDIATE VALUES FOR DISPLAY ==========
        
        double latencyMs = targetLatencyMs + captureLatencyMs;
        double latencySeconds = latencyMs / 1000.0;
        double totalTime = dtseconds + latencySeconds;
        
        Translation2d robotRelativeMovement = new Translation2d(
            vxMetersPerSec * totalTime,
            vyMetersPerSec * totalTime
        );
        
        double robotRelDistance = Math.sqrt(
            vxMetersPerSec * vxMetersPerSec + 
            vyMetersPerSec * vyMetersPerSec
        ) * totalTime;
        
        // ========== OUTPUT RESULTS ==========
        
        // Input echo
        SmartDashboard.putString("Test/Echo/Current Inputs", String.format(
            "Vx=%.2f Vy=%.2f Angle=%.0f° Latency=%.0fms DT=%.3fs",
            vxMetersPerSec, vyMetersPerSec, robotAngleDeg, latencyMs, dtseconds
        ));
        
        // Step-by-step calculations
        SmartDashboard.putNumber("Test/Calc/1-Target Latency (ms)", targetLatencyMs);
        SmartDashboard.putNumber("Test/Calc/2-Capture Latency (ms)", captureLatencyMs);
        SmartDashboard.putNumber("Test/Calc/3-Total Latency (ms)", latencyMs);
        SmartDashboard.putNumber("Test/Calc/4-Latency (seconds)", latencySeconds);
        SmartDashboard.putNumber("Test/Calc/5-DT (seconds)", dtseconds);
        SmartDashboard.putNumber("Test/Calc/6-Total Time (seconds)", totalTime);
        
        // Robot-relative movement (before rotation)
        SmartDashboard.putNumber("Test/RobotRelative/Delta X (m)", robotRelativeMovement.getX());
        SmartDashboard.putNumber("Test/RobotRelative/Delta Y (m)", robotRelativeMovement.getY());
        SmartDashboard.putNumber("Test/RobotRelative/Distance (m)", robotRelativeMovement.getNorm());
        SmartDashboard.putNumber("Test/RobotRelative/Angle (deg)", 
            Math.toDegrees(Math.atan2(robotRelativeMovement.getY(), robotRelativeMovement.getX())));
        
        // Field-relative movement (YOUR FUNCTION'S OUTPUT)
        SmartDashboard.putNumber("Test/Output/Field Delta X (m)", result.getX());
        SmartDashboard.putNumber("Test/Output/Field Delta Y (m)", result.getY());
        SmartDashboard.putNumber("Test/Output/Field Distance (m)", result.getNorm());
        SmartDashboard.putNumber("Test/Output/Field Angle (deg)", 
            Math.toDegrees(Math.atan2(result.getY(), result.getX())));
        
        // Verification formulas
        SmartDashboard.putString("Test/Verify/Speed", String.format(
            "Speed = √(Vx²+Vy²) = √(%.2f²+%.2f²) = %.3f m/s",
            vxMetersPerSec, vyMetersPerSec,
            Math.sqrt(vxMetersPerSec*vxMetersPerSec + vyMetersPerSec*vyMetersPerSec)
        ));
        
        SmartDashboard.putString("Test/Verify/Distance", String.format(
            "Distance = Speed × Time = %.3f × %.4f = %.4f m",
            Math.sqrt(vxMetersPerSec*vxMetersPerSec + vyMetersPerSec*vyMetersPerSec),
            totalTime,
            robotRelDistance
        ));
        
        // Status
        SmartDashboard.putBoolean("Test/Status/Running", true);
        SmartDashboard.putString("Test/Status/Camera Table", testCamera.getTableName());
        SmartDashboard.putString("Test/Status/Last Update", java.time.LocalTime.now().toString());
    }
}