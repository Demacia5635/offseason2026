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
import frc.robot.kinematics.KinematicsConstants.KinematicsConfig;

import static frc.robot.kinematics.KinematicsConstants.*;


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
    

    // Constants for Udi Velocities Limiter
    final double minV = 0.01; // slower is 0
    final double maxRadialA = 6.0; // centrifugal force
    final double maxLinearA = 10.0; // normal acceleration
    final double CT = 0.02; // cycle time
    final double maxDeltaV = maxLinearA * CT; // max velocity change in 1 cycle
    final double maxFastTurnAngle = maxRadialA / maxLinearA; // if heading change is lower than this value - don't slow - accelerate to target velocity
    final double minReverseAngle = Math.PI - maxFastTurnAngle; // if heading is bigger than this - deaccelerate and turn to reverse (optimize)
    final double maxR = 1.0; // if need to change direction and fast - reduce velocity to this radius
    final double maxRotationV = Math.sqrt(maxRadialA * maxR); // max velocity to use the preferred radius

    private ChassisSpeeds limitVelocitiesUdi(ChassisSpeeds wantedSpeeds, ChassisSpeeds currentSpeeds){
        double currentV = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        double wantedV =  Math.hypot(wantedSpeeds.vxMetersPerSecond, wantedSpeeds.vyMetersPerSecond);
        if(currentV < minV) { // we are standing
            if(wantedV < minV) { // target is standing
                return new ChassisSpeeds(0,0,wantedSpeeds.omegaRadiansPerSecond);
            } else { // target is moving
                // we are moving to the required heading and accelerating
                // calculate the ratio that we will do in this cycle
                double ratio = MathUtil.clamp(wantedV, currentV, currentV + maxDeltaV) / wantedV;
                return new ChassisSpeeds(wantedSpeeds.vxMetersPerSecond *ratio, wantedSpeeds.vyMetersPerSecond * ratio, wantedSpeeds.omegaRadiansPerSecond);
            }
        }
        if(wantedV < minV) { // target is stop
            // just deaccelrate to 0
            double ratio = Math.max(currentV-maxDeltaV, wantedV) / currentV;
            return new ChassisSpeeds(currentSpeeds.vxMetersPerSecond *ratio, currentSpeeds.vyMetersPerSecond * ratio, wantedSpeeds.omegaRadiansPerSecond);
        }
        // we are moving and target is moving
        double currentAngle = Math.atan2(currentSpeeds.vyMetersPerSecond, currentSpeeds.vxMetersPerSecond);
        double targetAngle = Math.atan2(wantedSpeeds.vyMetersPerSecond, wantedSpeeds.vxMetersPerSecond);
        double alpha = MathUtil.angleModulus(targetAngle - currentAngle);
        double targetV = wantedV;

        if(Math.abs(alpha) < maxFastTurnAngle) { // small heading change
            // accelerate to target v
            targetV = MathUtil.clamp(targetV, currentV - maxDeltaV, currentV + maxDeltaV);
        } else if(Math.abs(alpha) > minReverseAngle) { // optimization - deaccdelerate and turn the other way
            targetV = currentV - maxDeltaV;
            if(alpha > minReverseAngle) {
                alpha = alpha - Math.PI;
            } else {
                alpha = alpha + Math.PI;
            }
        } else  { // we slow to a good heading change velocity
            targetV = MathUtil.clamp(Math.min(maxRotationV, targetV), currentV - maxDeltaV, currentV + maxDeltaV);
        }
        if(targetV < minV) {
            return new ChassisSpeeds(0, 0, wantedSpeeds.omegaRadiansPerSecond);
        }
        // calculate the maximum heading change using the target velocity and allowed radial acceleration
        double maxAngleChange = maxRadialA / targetV * CT;
        // set the target angle
        targetAngle = MathUtil.clamp(targetAngle, currentAngle - maxAngleChange, currentAngle + maxAngleChange);
        // return the speeds - using target velocity and target angle
        return new ChassisSpeeds(targetV * Math.cos(targetAngle), targetV*Math.sin(targetAngle), wantedSpeeds.omegaRadiansPerSecond);
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