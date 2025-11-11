// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.UtilsForChassis;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.UtilsVision.Camera;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/** 
 * Calculates robot position changes accounting for vision latency.
 * This compensates for the delay between when vision data was captured
 * and when it's processed.
 */
public class CalculatePositionAndAngle {
    private double latency;
    private Supplier<ChassisSpeeds> speeds;
    private Supplier<Rotation2d> getRobotAngle;
    private double dtseconds;
    private Camera camera;
    private NetworkTable Table;
    private Supplier<Pose2d> currentPose2d;

    public CalculatePositionAndAngle(Supplier<Pose2d> currentPos){

    }
    public CalculatePositionAndAngle(){

    }

    public Translation2d computeDeltaPosition(Camera camera,Supplier<ChassisSpeeds> speeds, Supplier<Rotation2d> getRobotAngle, double dtseconds) {
        this.camera = camera;
        Table = NetworkTableInstance.getDefault().getTable(camera.getTableName());
        latency = Table.getEntry("tl").getDouble(0.0) + Table.getEntry("cl").getDouble(0.0);

        Translation2d relativeMovement = new Translation2d(
                speeds.get().vxMetersPerSecond * dtseconds+latency,
                speeds.get().vyMetersPerSecond * dtseconds+latency
        );
        
        return relativeMovement.rotateBy(getRobotAngle.get());
    }
}
