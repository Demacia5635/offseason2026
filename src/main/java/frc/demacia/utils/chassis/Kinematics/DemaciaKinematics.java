
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

    public DemaciaKinematics(Translation2d[] modulePositionOnTheRobot) {
        this.startRobotPosition = Pose2d.kZero;
        this.modulePositionOnTheRobot = modulePositionOnTheRobot;
    }

    public SwerveModuleState[] toSwerveModuleState(ChassisSpeeds wantedSpeeds, ChassisSpeeds currentSpeeds) {

        ChassisSpeeds limitedSpeeds = limitSpeeds(wantedSpeeds, currentSpeeds);

        return convertSpeedsToStates(limitedSpeeds);

    }

    private ChassisSpeeds limitSpeeds(ChassisSpeeds wantedSpeeds, ChassisSpeeds currentSpeeds) {
        // Translation2d limitedRadialVelocities = limitRadialVelocities(wantedSpeeds,
        // currentSpeeds);

        // ChassisSpeeds nextWantedSpeeds = new ChassisSpeeds(
        // limitedRadialVelocities.getX(),
        // limitedRadialVelocities.getY(),
        // wantedSpeeds.omegaRadiansPerSecond);

        return limitLinearSpeeds(wantedSpeeds, currentSpeeds);
    }

    private Translation2d limitRadialVelocities(ChassisSpeeds wantedSpeeds, ChassisSpeeds currentSpeeds) {
        Translation2d currentSpeedsVector = new Translation2d(currentSpeeds.vxMetersPerSecond,
                currentSpeeds.vyMetersPerSecond);
        Translation2d wantedSpeedsVector = new Translation2d(wantedSpeeds.vxMetersPerSecond,
                wantedSpeeds.vyMetersPerSecond);

        Translation2d wantedAccel = wantedSpeedsVector.minus(currentSpeedsVector);
        return currentSpeedsVector.plus(new Translation2d(MAX_SKID_ACCEL * CYCLE_DT, wantedAccel.getAngle()));

    }

    private double limitLinearAccel(double maxVel, double currentVel) {
        return MAX_LINEAR_ACCEL * (1 - (Math.abs(currentVel) / maxVel));
    }

    private Rotation2d lastVelocityAngle = Rotation2d.kZero;


    
    private ChassisSpeeds limitLinearSpeeds(ChassisSpeeds wantedSpeeds, ChassisSpeeds currentSpeeds) {

        

        double wantedOmegaAccel = (wantedSpeeds.omegaRadiansPerSecond - currentSpeeds.omegaRadiansPerSecond) / CYCLE_DT;
        double limitedOmegaAccel = MathUtil.clamp(wantedOmegaAccel, -MAX_OMEGA_ACCEL, MAX_OMEGA_ACCEL);
        double wantedDeltaOmega = limitedOmegaAccel * CYCLE_DT;
        double wantedOmegaVelocity = currentSpeeds.omegaRadiansPerSecond + wantedDeltaOmega;

        

        double wantedSpeedsNorm = Math.hypot(wantedSpeeds.vxMetersPerSecond, wantedSpeeds.vyMetersPerSecond);
        double currentSpeedsNorm = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        double wantedSpeedsAngle = Utilities.angleFromTranslation2d(wantedSpeeds.vxMetersPerSecond, wantedSpeeds.vyMetersPerSecond);
        double currentSpeedsAngle = Utilities.angleFromTranslation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        
        
        if(wantedSpeedsNorm == 0 && currentSpeedsNorm == 0) return new ChassisSpeeds(0, 0, wantedOmegaVelocity);
        
        

        double wantedLinearAccel = Math.hypot(wantedSpeeds.vxMetersPerSecond - currentSpeeds.vxMetersPerSecond,
                wantedSpeeds.vyMetersPerSecond - currentSpeeds.vyMetersPerSecond) / CYCLE_DT;

        double maxLimitedAccel = limitLinearAccel(MAX_LINEAR_VELOCITY,
                Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond));
        double limitedAccel = MathUtil.clamp(wantedLinearAccel, -maxLimitedAccel, maxLimitedAccel);
        double wantedDeltaV = limitedAccel * CYCLE_DT;


        
        
        
        if(wantedSpeedsNorm == 0.0 && currentSpeedsNorm > 0.1){
            Translation2d velocity = new Translation2d(currentSpeedsNorm + wantedDeltaV, lastVelocityAngle);
            return new ChassisSpeeds(velocity.getX(), velocity.getY(), wantedOmegaVelocity);
        }

        

        lastVelocityAngle = Rotation2d.fromRadians(currentSpeedsAngle);
        return new ChassisSpeeds(currentSpeeds.vxMetersPerSecond + wantedDeltaV,
                currentSpeeds.vyMetersPerSecond + wantedDeltaV,
                wantedOmegaVelocity);
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
