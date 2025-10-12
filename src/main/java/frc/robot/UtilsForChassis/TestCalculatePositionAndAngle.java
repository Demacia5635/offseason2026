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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.UtilsVision.Camera;
import frc.robot.chassis.subsystems.Chassis;

import java.util.function.Supplier;

/**
 * Real robot test for CalculatePositionAndAngle.
 * Compares predicted position changes with actual odometry changes.
 * 
 * USAGE:
 * 1. Initialize in robotInit(): RobotPositionTest.initialize(camera, chassis, () -> 0.02)
 * 2. Call periodic(): RobotPositionTest.periodic(chassis.getPose())
 * 3. View results in NetworkTables under "PosTest/"
 */
public class TestCalculatePositionAndAngle {
    
    private static CalculatePositionAndAngle calculator;
    private static Camera camera;
    private static Supplier<ChassisSpeeds> speedSupplier;
    private static Supplier<Rotation2d> angleSupplier;
    private static Supplier<Double> dtSupplier;
    
    // Tracking variables
    private static Pose2d lastPose = null;
    private static Translation2d lastPredictedDelta = null;
    private static double totalPredictedDistance = 0.0;
    private static double totalActualDistance = 0.0;
    private static double totalError = 0.0;
    private static int sampleCount = 0;
    private static double maxError = 0.0;
    private static double testStartTime = 0.0;
    
    // Test state
    private static boolean isRunning = false;
    private static boolean captureNextFrame = false;

    private static NetworkTable Table;
    private static double latency;

    
    /**
     * Initialize the test system with Chassis subsystem.
     * 
     * @param cam The camera to use for latency data
     * @param chassis The Chassis subsystem to get speeds and angle from
     * @param dt Supplier that returns the loop delta time (typically 0.02 for 20ms)
     */
    public static void initialize(Camera cam, Chassis chassis, Supplier<Double> dt) {
        camera = cam;
        speedSupplier = () -> chassis.getChassisSpeedsFieldRel();
        angleSupplier = () -> chassis.getGyroAngle();
        dtSupplier = dt;
        calculator = new CalculatePositionAndAngle();
        
        SmartDashboard.putString("PosTest/Instructions", 
            "Set 'PosTest/Control/Enable' to true to start test");
        SmartDashboard.putBoolean("PosTest/Control/Enable", false);
        SmartDashboard.putBoolean("PosTest/Control/Reset Stats", false);
    }
    
    /**
     * Initialize the test system with custom suppliers (for advanced use).
     * 
     * @param cam The camera to use for latency data
     * @param speeds Supplier that returns current chassis speeds
     * @param angle Supplier that returns current robot angle
     * @param dt Supplier that returns the loop delta time (typically 0.02 for 20ms)
     */
    public static void initializeCustom(Camera cam, 
                                   Supplier<ChassisSpeeds> speeds, 
                                   Supplier<Rotation2d> angle,
                                   Supplier<Double> dt) {
        camera = cam;
        speedSupplier = speeds;
        angleSupplier = angle;
        dtSupplier = dt;
        calculator = new CalculatePositionAndAngle();
        
        SmartDashboard.putString("PosTest/Instructions", 
            "Set 'PosTest/Control/Enable' to true to start test");
        SmartDashboard.putBoolean("PosTest/Control/Enable", false);
        SmartDashboard.putBoolean("PosTest/Control/Reset Stats", false);
    }
    
    /**
     * Call this every robot loop (typically in robotPeriodic or subsystem periodic).
     * 
     * @param currentPose The current robot pose from odometry
     */
    public static void periodic(Pose2d currentPose) {
        if (calculator == null) {
            SmartDashboard.putString("PosTest/Error", "Not initialized! Call initialize() first");
            return;
        }
        
        // Check if test is enabled
        boolean shouldRun = SmartDashboard.getBoolean("PosTest/Control/Enable", false);
        
        // Check for reset command
        if (SmartDashboard.getBoolean("PosTest/Control/Reset Stats", false)) {
            resetStatistics();
            SmartDashboard.putBoolean("PosTest/Control/Reset Stats", false);
        }
        
        // Handle state changes
        if (shouldRun && !isRunning) {
            startTest(currentPose);
        } else if (!shouldRun && isRunning) {
            stopTest();
        }
        
        if (!isRunning) {
            return;
        }
        
        // Run test logic
        runTestCycle(currentPose);
    }
    
    private static void startTest(Pose2d initialPose) {
        isRunning = true;
        lastPose = initialPose;
        captureNextFrame = true;
        testStartTime = Timer.getFPGATimestamp();
        SmartDashboard.putString("PosTest/Status", "RUNNING");
        SmartDashboard.putNumber("PosTest/Status/Start Time", testStartTime);
    }
    
    private static void stopTest() {
        isRunning = false;
        SmartDashboard.putString("PosTest/Status", "STOPPED");
    }
    
    private static void resetStatistics() {
        totalPredictedDistance = 0.0;
        totalActualDistance = 0.0;
        totalError = 0.0;
        maxError = 0.0;
        sampleCount = 0;
        SmartDashboard.putString("PosTest/Stats/Status", "RESET");
    }
    
    private static void runTestCycle(Pose2d currentPose) {
        double dt = dtSupplier.get();
        
        // Get current state
        ChassisSpeeds currentSpeeds = speedSupplier.get();
        Rotation2d currentAngle = angleSupplier.get();
        
        // Compute predicted delta from the function
        Translation2d predictedDelta = calculator.computeDeltaPosition(
            camera,
            speedSupplier,
            angleSupplier,
            dt
        );
        
        // Calculate actual delta from odometry
        Translation2d actualDelta = null;
        if (lastPose != null && !captureNextFrame) {
            actualDelta = currentPose.getTranslation().minus(lastPose.getTranslation());
            
            // Update statistics
            updateStatistics(predictedDelta, actualDelta);
        }
        
        // Output current frame data
        outputCurrentFrame(currentSpeeds, currentAngle, predictedDelta, actualDelta, dt);
        
        // Update for next cycle
        lastPose = currentPose;
        lastPredictedDelta = predictedDelta;
        captureNextFrame = false;
    }
    
    private static void updateStatistics(Translation2d predicted, Translation2d actual) {
        double predictedDist = predicted.getNorm();
        double actualDist = actual.getNorm();
        double error = predicted.minus(actual).getNorm();
        
        totalPredictedDistance += predictedDist;
        totalActualDistance += actualDist;
        totalError += error;
        sampleCount++;
        
        if (error > maxError) {
            maxError = error;
        }
        
        // Output statistics
        SmartDashboard.putNumber("PosTest/Stats/Sample Count", sampleCount);
        SmartDashboard.putNumber("PosTest/Stats/Avg Error (m)", totalError / sampleCount);
        SmartDashboard.putNumber("PosTest/Stats/Max Error (m)", maxError);
        SmartDashboard.putNumber("PosTest/Stats/Total Predicted (m)", totalPredictedDistance);
        SmartDashboard.putNumber("PosTest/Stats/Total Actual (m)", totalActualDistance);
        SmartDashboard.putNumber("PosTest/Stats/Distance Diff (m)", 
            Math.abs(totalPredictedDistance - totalActualDistance));
        
        // Error percentage
        if (totalActualDistance > 0.01) {
            double errorPercent = (totalError / totalActualDistance) * 100.0;
            SmartDashboard.putNumber("PosTest/Stats/Avg Error (%)", errorPercent);
        }
        
        // Test duration
        double duration = Timer.getFPGATimestamp() - testStartTime;
        SmartDashboard.putNumber("PosTest/Stats/Test Duration (s)", duration);
    }
    
    private static void outputCurrentFrame(ChassisSpeeds speeds, 
                                           Rotation2d angle, 
                                           Translation2d predicted,
                                           Translation2d actual,
                                           double dt) {

        Table = NetworkTableInstance.getDefault().getTable(camera.getTableName());
        latency = Table.getEntry("tl").getDouble(0.0) + Table.getEntry("cl").getDouble(0.0); 
                                            
        // Current robot state
        SmartDashboard.putNumber("PosTest/Current/Vx (m/s)", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("PosTest/Current/Vy (m/s)", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("PosTest/Current/Omega (rad/s)", speeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("PosTest/Current/Robot Angle (deg)", angle.getDegrees());
        SmartDashboard.putNumber("PosTest/Current/DT (s)", dt);
        
        double speedMagnitude = Math.sqrt(
            speeds.vxMetersPerSecond * speeds.vxMetersPerSecond +
            speeds.vyMetersPerSecond * speeds.vyMetersPerSecond
        );
        SmartDashboard.putNumber("PosTest/Current/Speed (m/s)", speedMagnitude);
        
        // Predicted movement
        SmartDashboard.putNumber("PosTest/Predicted/Delta X (m)", predicted.getX());
        SmartDashboard.putNumber("PosTest/Predicted/Delta Y (m)", predicted.getY());
        SmartDashboard.putNumber("PosTest/Predicted/Distance (m)", predicted.getNorm());
        SmartDashboard.putNumber("PosTest/Predicted/Angle (deg)", 
            Math.toDegrees(Math.atan2(predicted.getY(), predicted.getX())));
        
        // Actual movement (if available)
        if (actual != null) {
            SmartDashboard.putNumber("PosTest/Actual/Delta X (m)", actual.getX());
            SmartDashboard.putNumber("PosTest/Actual/Delta Y (m)", actual.getY());
            SmartDashboard.putNumber("PosTest/Actual/Distance (m)", actual.getNorm());
            SmartDashboard.putNumber("PosTest/Actual/Angle (deg)", 
                Math.toDegrees(Math.atan2(actual.getY(), actual.getX())));
            
            // Error calculation
            Translation2d errorVector = predicted.minus(actual);
            SmartDashboard.putNumber("PosTest/Error/Delta X (m)", errorVector.getX());
            SmartDashboard.putNumber("PosTest/Error/Delta Y (m)", errorVector.getY());
            SmartDashboard.putNumber("PosTest/Error/Magnitude (m)", errorVector.getNorm());
            
            // Error percentage
            if (actual.getNorm() > 0.001) {
                double errorPercent = (errorVector.getNorm() / actual.getNorm()) * 100.0;
                SmartDashboard.putNumber("PosTest/Error/Percent (%)", errorPercent);
            }
        }
        
        // Camera latency info
        SmartDashboard.putNumber("PosTest/Camera/Total Latency (ms)", 
            latency * 1000.0);
        SmartDashboard.putString("PosTest/Camera/Name", camera.getTableName());
    }
    
    /**
     * Get the current average error for programmatic use.
     * @return Average error in meters, or 0 if no samples
     */
    public static double getAverageError() {
        return sampleCount > 0 ? totalError / sampleCount : 0.0;
    }
    
    /**
     * Check if the test is currently running.
     * @return true if test is active
     */
    public static boolean isTestRunning() {
        return isRunning;
    }
}