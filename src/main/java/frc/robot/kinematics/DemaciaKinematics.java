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

        SmartDashboard.putNumber("CurrentVel/x", currentVel.getX());
        SmartDashboard.putNumber("CurrentVel/y", currentVel.getY());
        SmartDashboard.putNumber("WantedVel/x", wantedVel.getX());
        SmartDashboard.putNumber("WantedVel/y", wantedVel.getY());
        Translation2d wantedAccel = (wantedVel.minus(currentVel)).div(CYCLE_DT);
        // wantedAccel = limitTiltAccel(wantedAccel);
        wantedAccel = limitAccel(wantedAccel);
        Translation2d deltaV = wantedAccel.times(CYCLE_DT);
        SmartDashboard.putNumber("Delta v/x", deltaV.getX());
        SmartDashboard.putNumber("Delta v/y", deltaV.getY());
        
        return new ChassisSpeeds(currentSpeeds.vxMetersPerSecond + deltaV.getX(), currentSpeeds.vyMetersPerSecond + deltaV.getY(), wantedSpeeds.omegaRadiansPerSecond);

    }


    private Translation2d limitAccel(Translation2d wantedAccel){
        SmartDashboard.putNumber("pre wantedACc/x", wantedAccel.getX());
        SmartDashboard.putNumber("pre wantedACc/y", wantedAccel.getY());
        wantedAccel = limitTiltAccel(wantedAccel);

        return wantedAccel;


    }

    private Translation2d limitTiltAccel(Translation2d wantedAccel){
        double frontAccel = MathUtil.clamp(wantedAccel.getX(), -config.MAX_FRONT_ACCEL(), config.MAX_FRONT_ACCEL());
        SmartDashboard.putNumber("Front Accel", frontAccel);
        double sideAccel = MathUtil.clamp(wantedAccel.getY(), -config.MAX_SIDE_ACCEL(), config.MAX_SIDE_ACCEL());
        return new Translation2d(frontAccel, sideAccel);
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