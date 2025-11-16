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
    private Pose2d startRobotPosition;
    private Translation2d[] modulePositionOnTheRobot;
    private Translation2d velocityVector;
    private double omega;
    private double Vx;
    private double Vy;
    private double Mi;
    private double moduleAngleFromRobot;
    private double moduleCurrentAngle;

    public KinematicsNew(Translation2d[] modulePositionOnTheRobot){
        this.startRobotPosition = Pose2d.kZero;
        this.modulePositionOnTheRobot = modulePositionOnTheRobot;
    }

    public SwerveModuleState[] moduleStates(ChassisSpeeds chassisSpeeds){
        omega = chassisSpeeds.omegaRadiansPerSecond;
        for(int i = 0; i < 4; i++){
            Vx = chassisSpeeds.vxMetersPerSecond;
            Vy = chassisSpeeds.vyMetersPerSecond;
            Mi = modulePositionOnTheRobot[i].getNorm();
            moduleAngleFromRobot = modulePositionOnTheRobot[i].getAngle().getRadians();
            moduleCurrentAngle = startRobotPosition.getRotation().getRadians();
            velocityVector = new Translation2d(
                Vx - omega * Mi * Math.cos(moduleCurrentAngle + omega * 0.02 + moduleAngleFromRobot),
                Vy - omega * Mi * Math.sin(moduleCurrentAngle + omega * 0.02 + moduleAngleFromRobot)
            );
            swerveStates[i] = new SwerveModuleState(velocityVector.getNorm(), velocityVector.getAngle());
        }
        return swerveStates;
    }
}
