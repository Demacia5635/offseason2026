// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kinematics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class KinematicsNew {

    private SwerveModuleState[] swerveStates = new SwerveModuleState[4];
    private final double MAX_ALLOWED_MODULE_VELOCITY = 3;
    private Pose2d startRobotPosition;
    private Translation2d[] modulePositionOnTheRobot;
    private Translation2d velocityVector;
    private double omega;
    private double Vx;
    private double Vy;
    private double Mi;
    private final double MIN_VELOCITY = 0.03;
    private double moduleAngleFromRobot;
    private double moduleCurrentAngle;
    private double maxModuleVelocity;

    public KinematicsNew(Translation2d[] modulePositionOnTheRobot) {
        this.startRobotPosition = Pose2d.kZero;
        this.modulePositionOnTheRobot = modulePositionOnTheRobot;
    }

    public SwerveModuleState[] moduleStates(ChassisSpeeds chassisSpeeds) {
        omega = chassisSpeeds.omegaRadiansPerSecond;
        Vx = chassisSpeeds.vxMetersPerSecond;
        Vy = chassisSpeeds.vyMetersPerSecond;
        maxModuleVelocity = 0;

        for (int i = 0; i < 4; i++) {
            Mi = modulePositionOnTheRobot[i].getNorm();
            moduleAngleFromRobot = modulePositionOnTheRobot[i].getAngle().getRadians();
            moduleCurrentAngle = startRobotPosition.getRotation().getRadians();
            velocityVector = new Translation2d(
                    Vx + omega * Mi * Math.sin(moduleCurrentAngle + omega * 0.02 + moduleAngleFromRobot),
                    Vy - omega * Mi * Math.cos(moduleCurrentAngle + omega * 0.02 + moduleAngleFromRobot));
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