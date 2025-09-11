package frc.robot.Drive;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc.Demacia.Geometry.Pose2d;
import frc.Demacia.Geometry.Rotation2d;
import frc.Demacia.Geometry.Translation2d;
import frc.Demacia.utils.StatusSignalData;
import frc.Demacia.utils.Utilities;

public class ICRKinematics {
    final Translation2d[] modulePositions;
    final int N;
    SwerveModuleState[] currentStates;
    SwerveModulePosition[] currentPositions;
    SwerveModuleState[] lastStates;
    SwerveModulePosition[] lastPositions;

    SwerveModuleState[] velocities;

    Pose2d pose;
    double lastUpdateTime;
    StatusSignalData<Angle> gyroSignal;
    double lastHeading = 0;
    double currentHeading = 0;
    double gyroOffset = 0;

    ICRData[] icrData;

    public static final double VELOCITY_DEADBAND = 0.01;

    public ICRKinematics(Translation2d[] modulePositions, Pose2d initialPose, SwerveModuleState[] currentStates, SwerveModulePosition[] currentPositions, StatusSignalData<Angle> gyroSignal) {
        this.modulePositions = modulePositions;
        N = this.modulePositions.length;
        this.pose = new Pose2d(initialPose.getX(), initialPose.getY(),new Rotation2d(initialPose.getRotation().getRadians()));
        this.currentStates = new SwerveModuleState[modulePositions.length];
        this.currentPositions = new SwerveModulePosition[modulePositions.length];
        this.lastPositions = new SwerveModulePosition[currentPositions.length];
        this.lastStates = new SwerveModuleState[currentPositions.length];


        this.lastUpdateTime = Timer.getFPGATimestamp();
        this.gyroSignal = gyroSignal;
        currentHeading = initialPose.getRotation().getRadians();        
        lastHeading = currentHeading;
        gyroOffset = currentHeading - lastHeading;
        
        velocities = new SwerveModuleState[N];
        for (int i = 0; i < N; i++) {
            lastStates[i] = new SwerveModuleState(currentStates[i].speedMetersPerSecond, new Rotation2d(currentStates[i].angle.getRadians()));
            lastPositions[i] = new SwerveModulePosition(currentPositions[i].distanceMeters, new Rotation2d(currentPositions[i].angle.getRadians()));
            velocities[i] = new SwerveModuleState(0, new Rotation2d());
        }
        int n = 0;
        for(int i = 0; i < N; i++) {
            for(int j = i; j < N; j++) {
                icrData[n++] = new ICRData(modulePositions[i], modulePositions[j], velocities[i], velocities[j]);
            }
        }
    }

    private void copyCurrentToLast() {
        for(int i = 0; i < lastPositions.length; i++) {
            lastPositions[i].distanceMeters = currentPositions[i].distanceMeters;
//            lastPositions[i].angle.set(lastPositions[i].angle.getRadians());
            lastStates[i].speedMetersPerSecond = currentStates[i].speedMetersPerSecond;
//            lastStates[i].angle.set(currentStates[i].angle.getRadians());
        }
        lastHeading = currentHeading;
    }
    private void calculateVelocities(double dt) {
        for(int i = 0; i < modulePositions.length; i++) {
            velocities[i].speedMetersPerSecond = Utilities.deadband(
                (currentStates[i].speedMetersPerSecond + lastStates[i].speedMetersPerSecond + 
                (currentPositions[i].distanceMeters - lastPositions[i].distanceMeters)/dt)/3, VELOCITY_DEADBAND);
//            velocities[i].angle.set((currentStates[i].angle.getRadians() + lastStates[i].angle.getRadians())/2);
        }
    }


    
    public Pose2d updatePose() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastUpdateTime;
        
        if (dt <= 0) {
            return pose;
        }
        currentHeading = gyroSignal.getValue().in(Radians) + gyroOffset;
        calculateVelocities(dt);
        for(ICRData icr : icrData) {
            icr.calculateICR();
        }
        
        Translation2d icr = calculateICR(dt);
        if(icr != null) {

        }
        Twist2d twist = calculateTwistFromICR(icr, dt);
        
        pose = pose.exp(twist);
        copyCurrentToLast();

        lastUpdateTime = currentTime;
        
        return pose;
    }
    
    private Translation2d calculateICR(double dt) {
        double x = 0;
        double y = 0;
        double nRadial = 0;
        int nSlip = 0;
        int nNoMove = 0;
        for(ICRData icr : icrData) {
            icr.calculateICR();
            switch (icr.type) {
                case RADIAL:
                    x += icr.ICR.getX();
                    y += icr.ICR.getY();
                    nRadial ++;
                    break;
                case NO_MOVE:
                    nNoMove++;
                    break;
                case SLIP:
                    nSlip ++;
                    break;
                default:
                    break;
            }
        }
        if(nSlip > 3 || nNoMove > 3) 
            return null;
        if(nRadial > 2) {
            return new Translation2d(x / nRadial, y / nRadial);
        }        
        return null;
    }
    
    private Twist2d calculateTwistFromICR(Translation2d icr, double dt) {
        if (icr.getNorm() > 100) {
            return calculateLinearTwist(currentPositions, currentHeading, dt);
        }
        
        double gyroAngularChange = currentHeading - lastHeading;
        gyroAngularChange = Math.atan2(Math.sin(gyroAngularChange), Math.cos(gyroAngularChange));
        
        double totalDeltaDistance = 0;
        int validModules = 0;
        
        for (int i = 0; i < modulePositions.length; i++) {
            double deltaDistance = currentPositions[i].distanceMeters - lastPositions[i].distanceMeters;
            if (Math.abs(deltaDistance) > 0.001) {
                totalDeltaDistance += deltaDistance;
                validModules++;
            }
        }
        
        if (validModules == 0) {
            return new Twist2d(0, 0, gyroAngularChange);
        }
        
        double avgDeltaDistance = totalDeltaDistance / validModules;
        Translation2d robotToICR = icr.minus(pose.getTranslation());
        double radiusToICR = robotToICR.getNorm();
        
        if (radiusToICR < 0.01) {
            return new Twist2d(0, 0, gyroAngularChange);
        }
        
        double icrAngularChange = avgDeltaDistance / radiusToICR;
        double finalAngularChange = 0.7 * icrAngularChange + 0.3 * gyroAngularChange;
        
        Translation2d centerOfRotation = pose.getTranslation().plus(robotToICR);
        double dx = centerOfRotation.getX() + radiusToICR * Math.cos(pose.getRotation().getRadians() + finalAngularChange) - pose.getX();
        double dy = centerOfRotation.getY() + radiusToICR * Math.sin(pose.getRotation().getRadians() + finalAngularChange) - pose.getY();
        
        return new Twist2d(dx, dy, finalAngularChange);
    }
    
    private Twist2d calculateLinearTwist(SwerveModulePosition[] currentPositions, double gyroHeading, double dt) {
        double gyroAngularChange = gyroHeading - lastHeading;
        gyroAngularChange = Math.atan2(Math.sin(gyroAngularChange), Math.cos(gyroAngularChange));
        
        double totalDx = 0, totalDy = 0;
        int validModules = 0;
        
        for (int i = 0; i < modulePositions.length; i++) {
            double deltaDistance = currentPositions[i].distanceMeters - lastPositions[i].distanceMeters;
            
            if (Math.abs(deltaDistance) > 0.001) {
//                Rotation2d moduleAngle = currentPositions[i].angle;
//                totalDx += deltaDistance * moduleAngle.getCos();
//                totalDy += deltaDistance * moduleAngle.getSin();
                validModules++;
            }
        }
        
        if (validModules == 0) {
            return new Twist2d(0, 0, gyroAngularChange);
        }
        
        return new Twist2d(totalDx / validModules, totalDy / validModules, gyroAngularChange);
    }
    
    public void resetPose(Pose2d newPose) {
        this.pose = newPose;
        this.lastHeading = newPose.getRotation().getRadians();
        this.lastUpdateTime = Timer.getFPGATimestamp();
    }
    
    public Pose2d getPose() {
        return pose;
    }
}