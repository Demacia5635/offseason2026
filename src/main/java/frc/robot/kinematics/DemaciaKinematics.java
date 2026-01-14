// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.kinematics;

import java.util.Arrays;

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
    private final double MIN_ALPHA = Math.toRadians(5);

    
    public DemaciaKinematics(Translation2d[] modulePositions){
        this.modulePositions = modulePositions;
    }

    public SwerveModuleState[] toSwerveModuleState(ChassisSpeeds s, SwerveModuleState[] currentStates){
        SwerveModuleState[] states = new SwerveModuleState[4];
        estimatedPose = new Pose2d(s.vxMetersPerSecond * DELTA_T, 
            s.vyMetersPerSecond * DELTA_T,
            new Rotation2d(s.omegaRadiansPerSecond * DELTA_T));

        if(Math.abs(s.vxMetersPerSecond) < 0.01 && Math.abs(s.vyMetersPerSecond) < 0.01){
            if(Math.abs(s.omegaRadiansPerSecond) < 0.01){
                for(int i = 0; i < states.length; i++){
                    states[i] = new SwerveModuleState(0, currentStates[i].angle);
                }
                return states;
            }
            for(int i = 0; i < states.length; i++){
                states[i] = new SwerveModuleState(s.omegaRadiansPerSecond * modulePositions[i].getNorm(),
                    modulePositions[i].getAngle().plus(new Rotation2d(Math.PI/2)));
            }
            return states;
        }
        if(Math.abs(s.omegaRadiansPerSecond) < 0.01){
            Arrays.fill(states, new SwerveModuleState(Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond), new Rotation2d(s.vxMetersPerSecond, s.vyMetersPerSecond)));
            return states;
        }

        for(int i = 0; i < states.length; i++){
            states[i] = calculateModuleState(currentStates[i], modulePositions[i]);
        }
        System.out.println("current state: " + currentStates[0]);
        System.out.println("wanted state: " + states[0]);
        System.out.println("Chassis speeds: " + s);
        Translation2d temp = calculateFromModuleToEstimatedModule(modulePositions[0]);
        System.out.println("FROM MOD TO ESTIM x: " + temp.getX() + " y: " + temp.getY() + " norm: " + temp.getNorm() + " angle: " + temp.getAngle());
        


        return states;

    }
    private Translation2d calculateFromModuleToEstimatedModule( Translation2d modulePositionOnRobot){
        Translation2d estimatedModulePosition = estimatedPose.getTranslation().plus(modulePositionOnRobot.rotateBy(estimatedPose.getRotation()));
        return estimatedModulePosition.minus(modulePositionOnRobot);
    }

    private SwerveModuleState calculateModuleState(SwerveModuleState currentState, Translation2d modulePositionOnRobot){
        

        
        Translation2d fromModuleToEstimatedModule = calculateFromModuleToEstimatedModule(modulePositionOnRobot);

        double alpha = currentState.angle.getRadians() - fromModuleToEstimatedModule.getAngle().getRadians();
       
        if(Math.abs(alpha) / DELTA_T < MIN_ALPHA) {
            return new SwerveModuleState(fromModuleToEstimatedModule.getNorm()/DELTA_T,
                fromModuleToEstimatedModule.getAngle());
        }

        Rotation2d wantedAngle = currentState.angle.minus(new Rotation2d(2 * alpha));
        

        double arcLength = (fromModuleToEstimatedModule.getNorm() * alpha ) /Math.sin(alpha);
        
        //double acceleration = 2 * (arcLength - (currentState.speedMetersPerSecond * DELTA_T))  * (1/T_SQUARED);
        //TODO: add a checking for when acceleration is too high

        //double wantedVelocity = currentState.speedMetersPerSecond + (acceleration * DELTA_T);
        double wantedVelocity = (arcLength * 100) - currentState.speedMetersPerSecond;
        return new SwerveModuleState(wantedVelocity, wantedAngle);


    }

}
