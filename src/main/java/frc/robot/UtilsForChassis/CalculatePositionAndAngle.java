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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
/** Add your docs here. */
public class CalculatePositionAndAngle {
    private double latency;
    private Supplier<ChassisSpeeds> speeds;
    private Supplier<Rotation2d> getRobotAngle;
    private double dtseconds = 1.0;


    public static Translation2d computeDeltaPosition(ChassisSpeeds speeds, double getRobotAngle, double dt) {

        // כמה מטרים הרובוט נע בצירים של הרובוט עצמו
        Translation2d relativeMovement = new Translation2d(
                speeds.vxMetersPerSecond * dt,
                speeds.vyMetersPerSecond * dt
        );
    
        // מסובב את התנועה לצירי השדה לפי הזווית של הרובוט
        return relativeMovement.rotateBy(new Rotation2d(getRobotAngle));
    }
}
