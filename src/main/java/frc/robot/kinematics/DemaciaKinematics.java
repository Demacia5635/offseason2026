// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kinematics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class DemaciaKinematics {
    private final Translation2d[] modulePositions;
    private Pose2d estimatedPose = Pose2d.kZero;
    private final double DELTA_T = 0.02;
    private final double T_SQUARED = DELTA_T * DELTA_T;
    private final double MIN_ALPHA = 5 * (Math.PI / 180);

    
    public DemaciaKinematics(Translation2d[] modulePositions){
        this.modulePositions = modulePositions;
    }

    public SwerveModuleState[] toSwerveModuleState(ChassisSpeeds s, SwerveModuleState[] currentStates){
        SwerveModuleState[] states = new SwerveModuleState[4];
        estimatedPose = new Pose2d(s.vxMetersPerSecond * DELTA_T, 
            s.vyMetersPerSecond * DELTA_T,
            new Rotation2d(s.omegaRadiansPerSecond * DELTA_T));

        for(int i = 0; i < states.length; i++){
            states[i] = calculateModuleState(currentStates[i], modulePositions[i]);
        }

        return states;

    }

    private SwerveModuleState calculateModuleState(SwerveModuleState currentState, Translation2d modulePositionOnRobot){
        Translation2d estimatedModulePosition = estimatedPose.getTranslation().plus(modulePositionOnRobot.rotateBy(estimatedPose.getRotation()));
        double alpha = currentState.angle.getRadians() - estimatedModulePosition.getAngle().getRadians();
        
        if(Math.abs(alpha) / DELTA_T < MIN_ALPHA) {
            return new SwerveModuleState(estimatedModulePosition.getNorm()/DELTA_T,
                estimatedModulePosition.getAngle());
        }

        Rotation2d wantedAngle = currentState.angle.plus(new Rotation2d(2 * alpha));

        double arcLength = (estimatedModulePosition.getNorm() * alpha ) /Math.sin(alpha);
        
        double acceleration = 2 * (arcLength - (currentState.speedMetersPerSecond * DELTA_T))  * (1/T_SQUARED);
        //TODO: add a checking for when acceleration is too high

        double wantedVelocity = currentState.speedMetersPerSecond + (acceleration * DELTA_T);

        return new SwerveModuleState(wantedVelocity, wantedAngle);


    }

}
