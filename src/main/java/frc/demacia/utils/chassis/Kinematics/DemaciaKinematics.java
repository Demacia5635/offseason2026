
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.chassis.Kinematics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.demacia.utils.Utilities;

import static frc.demacia.utils.chassis.Kinematics.KinematicsConstants.*;

/** Add your docs here. */
public class DemaciaKinematics {

    private SwerveModuleState[] swerveStates = new SwerveModuleState[4];
    private Pose2d startRobotPosition;
    private Translation2d[] modulePositionOnTheRobot;
    private double lastAngle = 0;

    public DemaciaKinematics(Translation2d[] modulePositionOnTheRobot) {
        this.startRobotPosition = Pose2d.kZero;
        this.modulePositionOnTheRobot = modulePositionOnTheRobot;
    }

    public SwerveModuleState[] toSwerveModuleState(ChassisSpeeds wantedSpeeds, ChassisSpeeds currentSpeeds) {
        Translation2d wantedSpeedsVector = new Translation2d(wantedSpeeds.vxMetersPerSecond, wantedSpeeds.vyMetersPerSecond);
        Translation2d currentSpeedsVector = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        Translation2d wantedAccel = (wantedSpeedsVector.minus(currentSpeedsVector)).div(CYCLE_DT);
        wantedAccel = limitAccel(wantedAccel, currentSpeedsVector, wantedSpeedsVector);


        if(wantedSpeedsVector.getNorm() < 0.05 && currentSpeedsVector.getNorm() < 0.05){
            if(Math.abs(wantedSpeeds.omegaRadiansPerSecond) < 0.1) return convertSpeedsToStates(new ChassisSpeeds());
            return convertSpeedsToStates(new ChassisSpeeds(0, 0, wantedSpeeds.omegaRadiansPerSecond));


        } 
        ChassisSpeeds limitedSpeeds = new ChassisSpeeds(
                currentSpeeds.vxMetersPerSecond + wantedAccel.getX() * CYCLE_DT,
                currentSpeeds.vyMetersPerSecond + wantedAccel.getY() * CYCLE_DT,
                wantedSpeeds.omegaRadiansPerSecond
        );
        return convertSpeedsToStates(limitedSpeeds);
    }
    
    private Translation2d limitAccel(Translation2d wantedAccel, Translation2d currentSpeedsVector, Translation2d wantedSpeedsVector){
        // //double maxForwardAccel = MAX_LINEAR_ACCEL;
        // double maxForwardAccel = MAX_LINEAR_ACCEL * (1 - (currentSpeedsVector.getNorm() / MAX_LINEAR_VELOCITY));
        // return wantedAccel.div(wantedAccel.getNorm()).times(maxForwardAccel);
        if(wantedAccel.getNorm() > MAX_LINEAR_ACCEL) return new Translation2d(MAX_LINEAR_ACCEL, wantedAccel.getAngle());
        return wantedAccel;
    }

    

    private SwerveModuleState[] convertSpeedsToStates(ChassisSpeeds limitedSpeeds) {
        double omega = limitedSpeeds.omegaRadiansPerSecond;

        for (int i = 0; i < 4; i++) {
            double moduleAngleFromCenter = modulePositionOnTheRobot[i].getAngle().getRadians();
            double moduleCurrentAngle = startRobotPosition.getRotation().getRadians();
            Translation2d velocityVector = new Translation2d(
                    limitedSpeeds.vxMetersPerSecond + omega * modulePositionOnTheRobot[i].getNorm()
                            * Math.sin(moduleCurrentAngle + omega * 0.02 + moduleAngleFromCenter),
                    limitedSpeeds.vyMetersPerSecond - omega * modulePositionOnTheRobot[i].getNorm()
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

}
