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
        
        ChassisSpeeds limitedWantedVel = limitVelocities(wantedSpeeds, currentSpeeds);
        swerveStates = toSwerveModuleStates(limitedWantedVel);
        return swerveStates;
    }

    private ChassisSpeeds limitVelocities(ChassisSpeeds wantedSpeeds, ChassisSpeeds currentSpeeds){
        Translation2d currentVel = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        Translation2d wantedVel = new Translation2d(wantedSpeeds.vxMetersPerSecond, wantedSpeeds.vyMetersPerSecond);

        Translation2d wantedAccel = (wantedVel.minus(currentVel)).div(CYCLE_DT);
        // wantedAccel = limitTiltAccel(wantedAccel);
        
        Translation2d limitedAccel = limitAccel(wantedAccel);

        Translation2d deltaV = limitedAccel.times(CYCLE_DT);
      
        return new ChassisSpeeds(currentVel.getX() + deltaV.getX(), currentVel.getY() + deltaV.getY(), wantedSpeeds.omegaRadiansPerSecond);
    }


    private Translation2d limitAccel(Translation2d wantedAccel){
        wantedAccel = limitTiltAccel(wantedAccel);
        wantedAccel = limitSkidAccel(wantedAccel);

        return wantedAccel;


    }
    
    private Translation2d limitSkidAccel(Translation2d wantedAccel){
        return KinematicsUtilities.limitVector(wantedAccel, config.MAX_SKID_ACCEL());
    }

    private Translation2d limitTiltAccel(Translation2d wantedAccel){
        double frontAccel = MathUtil.clamp(wantedAccel.getX(), -config.MAX_FRONT_ACCEL(), config.MAX_FRONT_ACCEL());
        double sideAccel = MathUtil.clamp(wantedAccel.getY(), -config.MAX_SIDE_ACCEL(), config.MAX_SIDE_ACCEL());
        return new Translation2d(frontAccel, sideAccel);
    }

    
    

    public ChassisSpeeds toChassisSpeeds(SwerveModuleState[] swerveStates, double omegaFromGyro){
        double sumVx = 0;
        double sumVy = 0;


        for(int i = 0; i < 4; i++){
            double angleFromCenter = modulePositionOnTheRobot[i].getAngle().getRadians();
            double distanceFromCenter = modulePositionOnTheRobot[i].getNorm();
            double currentAngle = swerveStates[i].angle.getRadians();
            double moduleVx = swerveStates[i].speedMetersPerSecond * Math.cos(currentAngle);
            double moduleVy = swerveStates[i].speedMetersPerSecond * Math.sin(currentAngle);

            double chassisVx = moduleVx - (omegaFromGyro * distanceFromCenter * Math.sin(currentAngle + (omegaFromGyro * 0.02) + angleFromCenter));
            double chassisVy = moduleVy + (omegaFromGyro * distanceFromCenter * Math.cos(currentAngle + (omegaFromGyro * 0.02) + angleFromCenter));

            sumVx += chassisVx;
            sumVy += chassisVy;
        }
        return new ChassisSpeeds(sumVx / 4.0, sumVy / 4.0, omegaFromGyro);
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