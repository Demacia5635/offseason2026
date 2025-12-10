package frc.robot.UtilsForChassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.UtilsVision.Camera;

import java.util.function.Supplier;

public class TestCalculatePositionAndAngle {

    private static CalculatePositionAndAngle calculator;
    private static Supplier<Pose2d> currentPoseSupplier;
    private static Supplier<ChassisSpeeds> speedSupplier;
    private static Supplier<Rotation2d> angleSupplier;
    private static Supplier<Double> dtSupplier;

    private static Pose2d lastPose = null;
    private static boolean isRunning = false;

    /**
     * Initialize the test system with the required suppliers.
     *
     * @param camera The camera to use for latency data
     * @param speeds Supplier that returns current chassis speeds
     * @param angle Supplier that returns current robot angle
     * @param pose Supplier that returns the current robot pose
     * @param dt Supplier that returns the loop delta time (typically 0.02 for 20ms)
     */
    public static void initialize(Camera camera,
                                  Supplier<ChassisSpeeds> speeds,
                                  Supplier<Rotation2d> angle,
                                  Supplier<Pose2d> pose,
                                  Supplier<Double> dt) {
        speedSupplier = speeds;
        angleSupplier = angle;
        currentPoseSupplier = pose;
        dtSupplier = dt;

        calculator = new CalculatePositionAndAngle(camera, speeds, angle, pose, dt.get());

        SmartDashboard.putBoolean("PosTest/Control/Enable", false);
        SmartDashboard.putBoolean("PosTest/Control/Reset Stats", false);
        SmartDashboard.putString("PosTest/Status", "Initialized");
    }

    /**
     * Call this periodically (e.g., in robotPeriodic) to run the test.
     */
    public static void periodic() {
        if (calculator == null) {
            SmartDashboard.putString("PosTest/Error", "Not initialized! Call initialize() first.");
            return;
        }

        boolean shouldRun = SmartDashboard.getBoolean("PosTest/Control/Enable", false);

        if (shouldRun && !isRunning) {
            startTest();
        } else if (!shouldRun && isRunning) {
            stopTest();
        }

        if (isRunning) {
            runTestCycle();
        }
    }

    private static void startTest() {
        isRunning = true;
        lastPose = currentPoseSupplier.get();
        SmartDashboard.putString("PosTest/Status", "Running");
    }

    private static void stopTest() {
        isRunning = false;
        SmartDashboard.putString("PosTest/Status", "Stopped");
    }

    private static void runTestCycle() {
        Pose2d currentPose = currentPoseSupplier.get();
        ChassisSpeeds currentSpeeds = speedSupplier.get();
        Rotation2d currentAngle = angleSupplier.get();
        double dt = dtSupplier.get();

        // Compute predicted delta position
        Translation2d predictedDelta = calculator.computeDeltaPosition();

        // Compute actual delta position
        Translation2d actualDelta = null;
        if (lastPose != null) {
            actualDelta = currentPose.getTranslation().minus(lastPose.getTranslation());
        }

        // Display results on SmartDashboard
        SmartDashboard.putNumber("PosTest/Current/Vx (m/s)", currentSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("PosTest/Current/Vy (m/s)", currentSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("PosTest/Current/Omega (rad/s)", currentSpeeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("PosTest/Current/Robot Angle (deg)", currentAngle.getDegrees());
        SmartDashboard.putNumber("PosTest/Current/DT (s)", dt);

        if (predictedDelta != null) {
            SmartDashboard.putNumber("PosTest/Predicted/Delta X (m)", predictedDelta.getX());
            SmartDashboard.putNumber("PosTest/Predicted/Delta Y (m)", predictedDelta.getY());
            SmartDashboard.putNumber("PosTest/Predicted/Distance (m)", predictedDelta.getNorm());
        }

        if (actualDelta != null) {
            SmartDashboard.putNumber("PosTest/Actual/Delta X (m)", actualDelta.getX());
            SmartDashboard.putNumber("PosTest/Actual/Delta Y (m)", actualDelta.getY());
            SmartDashboard.putNumber("PosTest/Actual/Distance (m)", actualDelta.getNorm());

            // Calculate error
            Translation2d error = predictedDelta.minus(actualDelta);
            SmartDashboard.putNumber("PosTest/Error/Delta X (m)", error.getX());
            SmartDashboard.putNumber("PosTest/Error/Delta Y (m)", error.getY());
            SmartDashboard.putNumber("PosTest/Error/Magnitude (m)", error.getNorm());
        }

        lastPose = currentPose;
    }
}