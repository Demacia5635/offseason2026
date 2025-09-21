// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.UtilsForChassis;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;
/** Add your docs here. */
public class CalculatePositionAndAngle {
    private double latency;
    private Supplier<ChassisSpeeds> speeds;
    private Supplier<Rotation2d> getRobotAngle;
    private double dtseconds = 0.02;

    public static Translation2d computeDeltaPosition(Supplier<ChassisSpeeds> speeds, Supplier<Rotation2d> getRobotAngle, double dtseconds) {

        Translation2d relativeMovement = new Translation2d(
                speeds.get().vxMetersPerSecond * dtseconds,
                speeds.get().vyMetersPerSecond * dtseconds
        );
        
        return relativeMovement.rotateBy(getRobotAngle.get());
    }
}
