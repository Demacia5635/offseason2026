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
import frc.demacia.utils.Log.LogManager;

import static frc.robot.kinematics.KinematicsConstants.*;

/** Add your docs here. */
public class DemaciaKinematics {

    private SwerveModuleState[] swerveStates = new SwerveModuleState[4];
    private Pose2d startRobotPosition;
    private Translation2d[] modulePositionOnTheRobot;
    private SwerveModuleState[] lastStates = new SwerveModuleState[4];

    public DemaciaKinematics(Translation2d[] modulePositionOnTheRobot) {
        this.startRobotPosition = Pose2d.kZero;
        this.modulePositionOnTheRobot = modulePositionOnTheRobot;
        for (int i = 0; i < 4; i++) {
            swerveStates[i] = new SwerveModuleState();
            lastStates[i] = new SwerveModuleState();
        }

    }

    // public SwerveModuleState[] udiTest(ChassisSpeeds wantedSpeeds, ChassisSpeeds
    // currentSpeeds) {
    // if(Math.abs(wantedSpeeds.vxMetersPerSecond) < 0.05 &&
    // Math.abs(wantedSpeeds.vyMetersPerSecond) < 0.05 &&
    // Math.abs(wantedSpeeds.omegaRadiansPerSecond)>0.01){
    // SwerveModuleState[] rotationStates = new SwerveModuleState[4];
    // for(int i = 0; i < 4; i++){
    // rotationStates[i] = new SwerveModuleState(wantedSpeeds.omegaRadiansPerSecond
    // * modulePositionOnTheRobot[i].getNorm(),
    // modulePositionOnTheRobot[i].getAngle().plus(Rotation2d.kCW_90deg));
    // }
    // return rotationStates;
    // }

    // if(KinematicsUtilities.isInRange(currentSpeeds, 0.03) &&
    // KinematicsUtilities.isInRange(wantedSpeeds, 0.03)){
    // for (SwerveModuleState swerveModuleState : lastStates) {
    // swerveModuleState.speedMetersPerSecond = 0;
    // };
    // return lastStates;
    // }
    // swerveStates = toSwerveModuleStates(wantedSpeeds);
    // lastStates = swerveStates;
    // return swerveStates;
    // }

    /**
     * transforms from chassis speeds to swerve module states (speeds in field
     * relative)
     * 
     * @param fieldRelWantedSpeeds  in field relative speeds
     * @param fieldRelCurrentSpeeds in field relative speeds
     * @param currentGyroAngle      in Rotation2d
     * 
     */
    public SwerveModuleState[] toSwerveModuleStatesWithLimit(ChassisSpeeds fieldRelWantedSpeeds,
            ChassisSpeeds fieldRelCurrentSpeeds, Rotation2d currentGyroAngle) {

        ChassisSpeeds limitedWantedVel = limitVelocities(fieldRelWantedSpeeds, fieldRelCurrentSpeeds);
        limitedWantedVel = ChassisSpeeds.fromFieldRelativeSpeeds(limitedWantedVel, currentGyroAngle);
        swerveStates = toSwerveModuleStates(limitedWantedVel);
        return swerveStates;
    }

    private ChassisSpeeds chassisFromRest(double currentV, double wantedV, ChassisSpeeds wantedSpeeds) {

        if (wantedV < MIN_VELOCITY) { // target is standing
            return new ChassisSpeeds(0, 0, wantedSpeeds.omegaRadiansPerSecond);
        } else { // target is moving
            // we are moving to the required heading and accelerating, no radial limit
            double ratio = MathUtil.clamp(wantedV, currentV, currentV + MAX_DELTA_V) / wantedV;
            return new ChassisSpeeds(wantedSpeeds.vxMetersPerSecond * ratio, wantedSpeeds.vyMetersPerSecond * ratio,
                    wantedSpeeds.omegaRadiansPerSecond);
        }
    }

    private double optimizeAngleChange(double alpha) {
        return alpha > MIN_REVERSE_ANGLE ? alpha - Math.PI : alpha + Math.PI;

    }

    private ChassisSpeeds limitVelocities(ChassisSpeeds wantedSpeeds, ChassisSpeeds currentSpeeds) {
        double currentVelocity = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        double wantedVelocity = Math.hypot(wantedSpeeds.vxMetersPerSecond, wantedSpeeds.vyMetersPerSecond);

        if (currentVelocity < MIN_VELOCITY) { // we are standing
            return chassisFromRest(currentVelocity, wantedVelocity, wantedSpeeds);
        }

        if (wantedVelocity < MIN_VELOCITY) { // target is stop
            // just deaccelrate to stop
            double ratio = Math.max(currentVelocity - MAX_DELTA_V, wantedVelocity) / currentVelocity;
            return new ChassisSpeeds(currentSpeeds.vxMetersPerSecond * ratio, currentSpeeds.vyMetersPerSecond * ratio,
                    wantedSpeeds.omegaRadiansPerSecond);
        }
        // we are moving and target is moving
        double currentVelocityHeading = Math.atan2(currentSpeeds.vyMetersPerSecond, currentSpeeds.vxMetersPerSecond);
        double targetVelocityHeading = Math.atan2(wantedSpeeds.vyMetersPerSecond, wantedSpeeds.vxMetersPerSecond);
        double velocityHeadingDiff = MathUtil.angleModulus(targetVelocityHeading - currentVelocityHeading);
        double targetVelocity = wantedVelocity;

        if (Math.abs(velocityHeadingDiff) < MAX_FAST_TURN_ANGLE) { // small heading change
            // accelerate to target v
            targetVelocity = MathUtil.clamp(targetVelocity, currentVelocity - MAX_DELTA_V, currentVelocity + MAX_DELTA_V);
        } else if (Math.abs(velocityHeadingDiff) > MIN_REVERSE_ANGLE) { // optimization - deaccdelerate and turn the other way

            targetVelocity = currentVelocity - MAX_DELTA_V;
            velocityHeadingDiff = optimizeAngleChange(velocityHeadingDiff);

        } else {
            targetVelocity = MathUtil.clamp(Math.min(MAX_ROTATION_VELOCITY, targetVelocity), currentVelocity - MAX_DELTA_V, currentVelocity + MAX_DELTA_V);
        }

        if (targetVelocity < MIN_VELOCITY) {
            return new ChassisSpeeds(0, 0, wantedSpeeds.omegaRadiansPerSecond);
        }
        // calculate the maximum heading change using the target velocity and allowed
        // radial acceleration
        double maxAngleChange = (MAX_RADIAL_ACCEL / targetVelocity) * CYCLE_DT;
        // set the target angle
        velocityHeadingDiff = MathUtil.clamp(velocityHeadingDiff, -maxAngleChange, maxAngleChange);
        targetVelocityHeading = currentVelocityHeading + velocityHeadingDiff;

        // return the speeds - using target velocity and target angle
        return new ChassisSpeeds(targetVelocity * Math.cos(targetVelocityHeading), targetVelocity * Math.sin(targetVelocityHeading),
                wantedSpeeds.omegaRadiansPerSecond);
    }

    public ChassisSpeeds toChassisSpeeds(SwerveModuleState[] swerveStates, double omegaFromGyro) {
        double sumVx = 0;
        double sumVy = 0;

        for (int i = 0; i < 4; i++) {
            double angleFromCenter = modulePositionOnTheRobot[i].getAngle().getRadians();
            double distanceFromCenter = modulePositionOnTheRobot[i].getNorm();
            double currentAngle = swerveStates[i].angle.getRadians();
            double moduleVx = swerveStates[i].speedMetersPerSecond * Math.cos(currentAngle);
            double moduleVy = swerveStates[i].speedMetersPerSecond * Math.sin(currentAngle);

            double chassisVx = moduleVx - (omegaFromGyro * distanceFromCenter
                    * Math.sin(currentAngle + (omegaFromGyro * 0.02) + angleFromCenter));
            double chassisVy = moduleVy + (omegaFromGyro * distanceFromCenter
                    * Math.cos(currentAngle + (omegaFromGyro * 0.02) + angleFromCenter));

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
            swerveStates[i] = new SwerveModuleState(velocityVector.getNorm(), new Rotation2d(
                    KinematicsUtilities.getAngleFromVector(velocityVector.getX(), velocityVector.getY())));
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