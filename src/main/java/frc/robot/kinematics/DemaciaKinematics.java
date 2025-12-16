// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kinematics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.demacia.utils.Utilities;

import static frc.robot.kinematics.KinematicsConstants.*;

/** Add your docs here. */
public class DemaciaKinematics {

    private SwerveModuleState[] swerveStates = new SwerveModuleState[4];
    private Pose2d startRobotPosition;
    private Translation2d[] modulePositionOnTheRobot;

    private ChassisSpeeds lastSpeeds;

    public DemaciaKinematics(Translation2d[] modulePositionOnTheRobot) {
        this.startRobotPosition = Pose2d.kZero;
        this.modulePositionOnTheRobot = modulePositionOnTheRobot;
        this.lastSpeeds = new ChassisSpeeds();

    }

    public SwerveModuleState[] toSwerveModuleState(ChassisSpeeds wantedSpeeds, ChassisSpeeds currentSpeeds){
        return moduleStates(limitLinearSpeeds(wantedSpeeds, currentSpeeds));

    }

    public SwerveModuleState[] moduleStates(ChassisSpeeds wantedSpeeds) {

        double omega = wantedSpeeds.omegaRadiansPerSecond;

        for (int i = 0; i < 4; i++) {
            double moduleAngleFromCenter = modulePositionOnTheRobot[i].getAngle().getRadians();
            double moduleCurrentAngle = startRobotPosition.getRotation().getRadians();
            Translation2d velocityVector = new Translation2d(
                    wantedSpeeds.vxMetersPerSecond + omega * modulePositionOnTheRobot[i].getNorm()
                            * Math.sin(moduleCurrentAngle + omega * 0.02 + moduleAngleFromCenter),
                    wantedSpeeds.vyMetersPerSecond - omega * modulePositionOnTheRobot[i].getNorm()
                            * Math.cos(moduleCurrentAngle + omega * 0.02 + moduleAngleFromCenter));
            swerveStates[i] = new SwerveModuleState(velocityVector.getNorm(), velocityVector.getAngle());
        }

        swerveStates = factorModuleVelocities(swerveStates);

        return swerveStates;
    }

    public SwerveModuleState[] factorModuleVelocities(SwerveModuleState[] swerveStates) {
        double maxVelocityCalculated = 0;
        for (int i = 0; i < swerveStates.length; i++) {
            double cur = Math.abs(swerveStates[i].speedMetersPerSecond);
            if (cur == 0)
                return swerveStates;
            if (cur > maxVelocityCalculated)
                maxVelocityCalculated = cur;
        }
        double factor = MAX_ALLOWED_MODULE_VELOCITY / maxVelocityCalculated;

        if (factor >= 1)
            return swerveStates;

        for (SwerveModuleState state : swerveStates) {
            state.speedMetersPerSecond = state.speedMetersPerSecond * factor;
        }
        return swerveStates;

    }

    private ChassisSpeeds limitLinearSpeeds(
            ChassisSpeeds wantedSpeeds,
            ChassisSpeeds currentSpeeds) {

        Translation2d limitedLinearVel = calculateLinearVel(
                wantedSpeeds.vxMetersPerSecond,
                wantedSpeeds.vyMetersPerSecond,
                currentSpeeds.vxMetersPerSecond,
                currentSpeeds.vyMetersPerSecond);

        return new ChassisSpeeds(
                limitedLinearVel.getX(),
                limitedLinearVel.getY(),
                wantedSpeeds.omegaRadiansPerSecond);

    }

    double lastAngle = 0;
    private Translation2d calculateLinearVel(double wantedSpeedsX, double wantedSpeedsY, double currentSpeedsX, double currentSpeedsY) {
        double wantedSpeedsNorm = Math.hypot(wantedSpeedsX, wantedSpeedsY);
        double currentSpeedsNorm = Math.hypot(currentSpeedsX, currentSpeedsY);
        double wantedSpeedsAngle = Math.atan2(wantedSpeedsY, wantedSpeedsX);
        double currentSpeedsAngle = Math.atan2(currentSpeedsY, currentSpeedsX);

        double maxDeltaV = config.MAX_LINEAR_ACCEL() * CYCLE_DT;
        double requiredLinearAccel = wantedSpeedsNorm - currentSpeedsNorm;

        if(wantedSpeedsNorm <= 0.05 && currentSpeedsNorm <= 0.05) return Translation2d.kZero;
        
        if(wantedSpeedsNorm <= 0.05 && currentSpeedsNorm > 0.1) 
            return new Translation2d(currentSpeedsNorm - maxDeltaV, 
            Rotation2d.fromRadians(lastAngle));

        lastAngle = currentSpeedsAngle;

        return new Translation2d(wantedSpeedsNorm + (maxDeltaV * Math.signum(requiredLinearAccel)), Rotation2d.fromRadians(wantedSpeedsAngle));

       



    }

}