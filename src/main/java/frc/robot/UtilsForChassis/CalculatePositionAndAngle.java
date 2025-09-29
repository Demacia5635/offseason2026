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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.UtilsVision.Camera;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;
/** Add your docs here. */
public class CalculatePositionAndAngle {
    private double latency;
    private Supplier<ChassisSpeeds> speeds;
    private Supplier<Rotation2d> getRobotAngle;
    private double dtseconds = 0.02;
    private Camera camera;
    public Pose2d pose;
    private NetworkTable Table;
    public boolean is3D;

    public Translation2d computeDeltaPosition(Camera camera,Supplier<ChassisSpeeds> speeds, Supplier<Rotation2d> getRobotAngle, double dtseconds) {
      
          latency = Table.getEntry("tl").getDouble(0.0) + Table.getEntry("cl").getDouble(0.0)*(10^3);
    
    Translation2d relativeMovement = new Translation2d(
            speeds.get().vxMetersPerSecond * dtseconds+latency,
            speeds.get().vyMetersPerSecond * dtseconds+latency
     );
        return relativeMovement.rotateBy(getRobotAngle.get());
        
    }
}

