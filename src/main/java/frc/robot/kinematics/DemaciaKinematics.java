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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.Utilities;
import frc.demacia.utils.Log.LogManager;
import frc.robot.kinematics.KinematicsConstants.KinematicsConfig;

import static edu.wpi.first.units.Units.derive;
import static frc.robot.kinematics.KinematicsConstants.*;

import java.util.spi.CurrencyNameProvider;

/** Add your docs here. */
public class DemaciaKinematics {

    private SwerveModuleState[] swerveStates = new SwerveModuleState[4];
    private Pose2d startRobotPosition;
    private Translation2d[] modulePositionOnTheRobot;
    private KinematicsConfig config;
    private double lastVelAngle;
    private SwerveModuleState[] lastStates = new SwerveModuleState[4]; 
    private final SwerveModuleState[] kZeroStates = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    public DemaciaKinematics(Translation2d[] modulePositionOnTheRobot) {
        this.startRobotPosition = Pose2d.kZero;
        this.modulePositionOnTheRobot = modulePositionOnTheRobot;
        for(int i = 0; i < 4; i++){
            swerveStates[i] = new SwerveModuleState();
            lastStates[i] = new SwerveModuleState();
        }
        this.config = KinematicsConstants.config;
        lastVelAngle = 0;

    }


    public SwerveModuleState[] toSwerveModuleStatesWithLimit(ChassisSpeeds wantedSpeeds, ChassisSpeeds currentSpeeds){
        if(KinematicsUtilities.isInRange(currentSpeeds, 0.05) && KinematicsUtilities.isInRange(wantedSpeeds, 0.05)) return kZeroStates;
        ChassisSpeeds limitedWantedVel = limitVelocities(wantedSpeeds, currentSpeeds);
        swerveStates = toSwerveModuleStates(limitedWantedVel);
        return swerveStates;
    }

    private ChassisSpeeds limitVelocities(ChassisSpeeds wantedSpeeds, ChassisSpeeds currentSpeeds){
        Translation2d currentVel = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        Translation2d wantedVel = new Translation2d(wantedSpeeds.vxMetersPerSecond, wantedSpeeds.vyMetersPerSecond);

        Translation2d wantedAccel = (wantedVel.minus(currentVel)).div(CYCLE_DT);
        LogManager.log("wanted accel pre: " + wantedAccel);
        LogManager.log("pre norm: " + wantedAccel.getNorm());
        // wantedAccel = limitTiltAccel(wantedAccel);
        wantedAccel = limitSkidAccel(wantedAccel);
        LogManager.log("wanted accel after: " + wantedAccel);
        
        LogManager.log("after norm: " + wantedAccel.getNorm());
        Translation2d deltaV = wantedAccel.times(CYCLE_DT);
        
        
        return new ChassisSpeeds(currentSpeeds.vxMetersPerSecond + deltaV.getX(), currentSpeeds.vyMetersPerSecond + deltaV.getY(), wantedSpeeds.omegaRadiansPerSecond);

    }
    private Translation2d limitSkidAccel(Translation2d wantedAccel){
        return KinematicsUtilities.limitVector(wantedAccel, config.MAX_SKID_ACCEL());
    }
    
    private Translation2d limitTiltAccel(Translation2d wantedAccel){
        double frontAccel = Math.min(wantedAccel.getX(), config.MAX_FRONT_ACCEL());
        double sideAccel = Math.min(wantedAccel.getY(), config.MAX_SIDE_ACCEL());
        return new Translation2d(frontAccel, sideAccel);
    }

    private Translation2d limitLinearVelocity(Translation2d currentVel, Translation2d wantedVel){
        double wantedSpeedsNorm = KinematicsUtilities.getNorm(wantedVel.getX(), wantedVel.getY());
        double currentSpeedsNorm = KinematicsUtilities.getNorm(currentVel.getX(), currentVel.getY());
        double wantedSpeedsAngle = KinematicsUtilities.getAngleFromVector(wantedVel.getX(), wantedVel.getY());
        double currentSpeedsAngle = KinematicsUtilities.getAngleFromVector(currentVel.getX(), currentVel.getY());

        if(KinematicsUtilities.isInRange(wantedSpeedsNorm, 0.05) && KinematicsUtilities.isInRange(currentSpeedsNorm, 0.05)) return Translation2d.kZero; //case for no movement
        if(KinematicsUtilities.isInRange(wantedSpeedsNorm, 0.05) && !KinematicsUtilities.isInRange(currentSpeedsNorm, 0.05)) 
            return new Translation2d(applyLinearLimit(currentSpeedsNorm, wantedSpeedsNorm), new Rotation2d(lastVelAngle)); //case for "gliding" to stoppage

        lastVelAngle = currentSpeedsAngle;
        return new Translation2d(applyLinearLimit(currentSpeedsNorm, wantedSpeedsNorm), new Rotation2d(wantedSpeedsAngle));

        
    }
    private double applyLinearLimit(double currentSpeedsNorm, double wantedSpeedsNorm){
        double wantedDeltaV = wantedSpeedsNorm - currentSpeedsNorm;
        if(Math.abs(wantedDeltaV) > MAX_DELTA_V) return currentSpeedsNorm + (MAX_DELTA_V * Math.signum(wantedDeltaV));
        
        return wantedSpeedsNorm;
    }

    
    


    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds wantedSpeeds) {

        
        double omega = wantedSpeeds.omegaRadiansPerSecond;

        for (int i = 0; i < 4; i++) {
            double moduleAngleFromCenter = modulePositionOnTheRobot[i].getAngle().getRadians();
            double moduleCurrentAngle = startRobotPosition.getRotation().getRadians();
            Translation2d velocityVector = new Translation2d(
                    wantedSpeeds.vxMetersPerSecond + omega * modulePositionOnTheRobot[i].getNorm()
                            * Math.sin(moduleCurrentAngle + omega * 0.02 + moduleAngleFromCenter),
                    wantedSpeeds.vyMetersPerSecond - omega * modulePositionOnTheRobot[i].getNorm()
                            * Math.cos(moduleCurrentAngle + omega * 0.02 + moduleAngleFromCenter));
            swerveStates[i] = new SwerveModuleState(velocityVector.getNorm(), new Rotation2d(KinematicsUtilities.getAngleFromVector(velocityVector.getX(), velocityVector.getY())));
        }

        swerveStates = factorModuleVelocities(swerveStates);

        return swerveStates;
    }

    private SwerveModuleState[] factorModuleVelocities(SwerveModuleState[] swerveStates) {
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