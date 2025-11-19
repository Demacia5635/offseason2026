// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kinematics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class DemaciaKinematics {

    private SwerveModuleState[] swerveStates = new SwerveModuleState[4];
    private final double MAX_ALLOWED_MODULE_VELOCITY = 3;
    private Pose2d startRobotPosition;
    private Translation2d[] modulePositionOnTheRobot;
    public DemaciaKinematics(Translation2d[] modulePositionOnTheRobot) {
        this.startRobotPosition = Pose2d.kZero;
        this.modulePositionOnTheRobot = modulePositionOnTheRobot;
    }

    public SwerveModuleState[] moduleStates(ChassisSpeeds chassisSpeeds) {
        double omega = chassisSpeeds.omegaRadiansPerSecond;
        
        for (int i = 0; i < 4; i++) {
            double moduleAngleFromCenter = modulePositionOnTheRobot[i].getAngle().getRadians();
            double moduleCurrentAngle = startRobotPosition.getRotation().getRadians();
            Translation2d velocityVector = new Translation2d(
                chassisSpeeds.vxMetersPerSecond + omega * modulePositionOnTheRobot[i].getNorm() * Math.sin(moduleCurrentAngle + omega * 0.02 + moduleAngleFromCenter),
                chassisSpeeds.vyMetersPerSecond - omega * modulePositionOnTheRobot[i].getNorm() * Math.cos(moduleCurrentAngle + omega * 0.02 + moduleAngleFromCenter));
            swerveStates[i] = new SwerveModuleState(velocityVector.getNorm(), velocityVector.getAngle());
        }

        swerveStates = factorModuleVelocities(swerveStates);

        return swerveStates;
    }

    public SwerveModuleState[] factorModuleVelocities(SwerveModuleState[] swerveStates) {
        double maxVelocityCalculated = 0;
        for (int i = 0; i < swerveStates.length; i++) {
            double cur = Math.abs(swerveStates[i].speedMetersPerSecond);
            if(cur == 0) return swerveStates;
            if (cur > maxVelocityCalculated) maxVelocityCalculated = cur;
        }
        double factor = MAX_ALLOWED_MODULE_VELOCITY / maxVelocityCalculated;

        if (factor >= 1)
            return swerveStates;

        for (SwerveModuleState state : swerveStates) {
            state.speedMetersPerSecond = state.speedMetersPerSecond * factor;
        }
        return swerveStates;

    }

}