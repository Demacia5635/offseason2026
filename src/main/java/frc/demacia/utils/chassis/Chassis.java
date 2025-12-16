// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.chassis;

import java.util.List;

import org.ejml.simple.SimpleMatrix;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Utilities;
import frc.demacia.utils.Sensors.Pigeon;
import frc.robot.kinematics.DemaciaKinematics;
import frc.robot.kinematics.DemaciaKinematics;

public class Chassis extends SubsystemBase {
  
  ChassisConfig chassisConfig;
  private SwerveModule[] modules;
  private Pigeon gyro;

  private DemaciaKinematics kinematics;
  private SwerveDriveKinematics kinematicsOld;
  private SwerveDrivePoseEstimator poseEstimator;
  private Field2d field;

  public Chassis(ChassisConfig chassisConfig) {
    this.chassisConfig = chassisConfig;
    modules = new SwerveModule[] {
      new SwerveModule(chassisConfig.frontLeftModuleConfig),
      new SwerveModule(chassisConfig.frontRightModuleConfig),
      new SwerveModule(chassisConfig.backLeftModuleConfig),
      new SwerveModule(chassisConfig.backRightModuleConfig),
    };
    gyro = new Pigeon(chassisConfig.pigeonConfig);
    kinematics = new DemaciaKinematics(new Translation2d[]
      {chassisConfig.frontLeftPosition,
      chassisConfig.frontRightPosition,
      chassisConfig.backLeftPosition,
      chassisConfig.backRightPosition}
      );
      kinematicsOld = new SwerveDriveKinematics(chassisConfig.frontLeftPosition,
      chassisConfig.frontRightPosition,
      chassisConfig.backLeftPosition,
      chassisConfig.backRightPosition);
    poseEstimator = new SwerveDrivePoseEstimator(kinematicsOld, getGyroAngle(), getModulePositions(), new Pose2d());

    SimpleMatrix std = new SimpleMatrix(new double[] { 0.02, 0.02, 0 });
    poseEstimator.setVisionMeasurementStdDevs(new Matrix<>(std));
    field = new Field2d();

    SmartDashboard.putData("chassis/setCoast", new InstantCommand(()->setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("chassis/Reset Gyro", new InstantCommand(()->setYaw(new Rotation2d())).ignoringDisable(true));
}

  public void checkElectronics() {
    for (SwerveModule module : modules) {
        module.checkElectronics();
    }
  }

  public void setNeutralMode(boolean isBrake) {
    for (SwerveModule module : modules) {
        module.setNeutralMode(isBrake);
    }
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }



  Translation2d lastWantedSpeeds = new Translation2d();
    public void setVelocitiesWithAccel(ChassisSpeeds wantedSpeeds){
        ChassisSpeeds currentSpeeds = getChassisSpeedsFieldRel();
        Translation2d limitedVelocitiesVector = calculateVelocity(wantedSpeeds.vxMetersPerSecond, wantedSpeeds.vyMetersPerSecond, currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);//lastWantedSpeeds);
        ChassisSpeeds limitedVelocities = new ChassisSpeeds(limitedVelocitiesVector.getX(), limitedVelocitiesVector.getY(), wantedSpeeds.omegaRadiansPerSecond);
        lastWantedSpeeds = limitedVelocitiesVector;
        setVelocities(limitedVelocities);

    }

    public void setVelocitiesTest(ChassisSpeeds speeds){
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroAngle());
        SwerveModuleState[] states = kinematics.toSwerveModuleState(speeds, getChassisSpeedsFieldRel());
        setModuleStates(states);
    }
    public void setVelocities(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroAngle());
        speeds = ChassisSpeeds.discretize(speeds, chassisConfig.cycleDt);
        SwerveModuleState[] states = kinematics.moduleStates(speeds);
        setModuleStates(states);
    }

    private double calculateLinearVelocity(double wantedSpeeds, double currentSpeeds) {
        double deltaV = wantedSpeeds - currentSpeeds;
        double maxDelta = chassisConfig.maxLinearAccel * chassisConfig.cycleDt;
        if(Math.abs(deltaV) > maxDelta){
            return currentSpeeds + (maxDelta * Math.signum(deltaV));
        }
        return wantedSpeeds;
        
    }

    public void setRobotRelSpeedsWithAccel(ChassisSpeeds speeds){
        ChassisSpeeds currentSpeeds = getChassisSpeedsRobotRel();

        Translation2d limitedVelocitiesVector = calculateVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);//lastWantedSpeeds);
        ChassisSpeeds limitedVelocities = new ChassisSpeeds(limitedVelocitiesVector.getX(), limitedVelocitiesVector.getY(), speeds.omegaRadiansPerSecond);
        
        setRobotRelVelocities(limitedVelocities);
    }

    double lastAngle = 0;
    private Translation2d calculateVelocity(double wantedSpeedsX, double wantedSpeedsY, double currentSpeedsX, double currentSpeedsY) {
        double wantedSpeedsNorm = Utilities.hypot(wantedSpeedsX, wantedSpeedsY);
        double currentSpeedsNorm = Utilities.hypot(currentSpeedsX, currentSpeedsY);
        double wantedSpeedsAngle = Utilities.angleFromTranslation2d(wantedSpeedsX, wantedSpeedsY);
        double currentSpeedsAngle = Utilities.angleFromTranslation2d(currentSpeedsX, currentSpeedsY);

        if(wantedSpeedsNorm == 0 && currentSpeedsNorm == 0) return Translation2d.kZero;

        if(currentSpeedsNorm <0.1){
            // LogManager.log("SMALL VEL");
            double v = MathUtil.clamp(wantedSpeedsNorm, 0, currentSpeedsNorm + chassisConfig.maxDeltaVelocity);
            return new Translation2d(v, Rotation2d.fromRadians(wantedSpeedsAngle));
        }

        if(wantedSpeedsNorm == 0 && currentSpeedsNorm > 0.1) return new Translation2d(calculateLinearVelocity(wantedSpeedsNorm, currentSpeedsNorm), Rotation2d.fromRadians(lastAngle));
        lastAngle = currentSpeedsAngle;
        double angleDiff = MathUtil.angleModulus(wantedSpeedsAngle - currentSpeedsAngle);
        double radius = currentSpeedsNorm / chassisConfig.maxOmegaVelocity;
        // LogManager.log("RADIUS: " + radius);
        if(Math.abs(angleDiff) < 0.6 || radius < chassisConfig.maxRadius){
            
            return new Translation2d(calculateLinearVelocity(wantedSpeedsNorm, currentSpeedsNorm), Rotation2d.fromRadians(wantedSpeedsAngle));
        }

        double velocity = Math.min(chassisConfig.maxVelocityToIgnoreRadius, Math.max(currentSpeedsNorm - (chassisConfig.maxDeltaVelocity), chassisConfig.minVelocity));
    //    LogManager.log("NEW VELOCITY: " + velocity);
        double radChange = Math.min(chassisConfig.maxOmegaVelocity, (velocity / chassisConfig.maxRadius) * chassisConfig.cycleDt);
        return new Translation2d(velocity, Rotation2d.fromRadians((radChange * Math.signum(angleDiff)) + currentSpeedsAngle));
        
    }

    public void setSteerPositions(double[] positions) {
        for (int i = 0; i < positions.length; i++) {
            modules[i].setSteerPosition(positions[i]);
        }
    }

    public void setSteerPower(double pow, int id) {
        modules[id].setSteerPower(pow);
    }

    public double getSteerVelocity(int id) {
        return modules[id].getSteerVel();
    }

    public double getSteerAcceleration(int id) {
        return modules[id].getSteerAccel();
    }

    public void setSteerPositions(double position) {
        setSteerPositions(new double[] { position, position, position, position });
    }

    public ChassisSpeeds getRobotRelVelocities() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(getChassisSpeedsRobotRel(), getGyroAngle());
    }

    public void setRobotRelVelocities(ChassisSpeeds speeds) {

        SwerveModuleState[] states = kinematics.moduleStates(speeds);
        setModuleStates(states);
    }

    public void setDriveVelocities(double[] velocities) {
        for (int i = 0; i < velocities.length; i++) {
            modules[i].setDriveVelocity(velocities[i]);
        }
    }

    public void setDriveVelocities(double velocity) {
        setDriveVelocities(new double[] { velocity, velocity, velocity, velocity });
    }

    public Rotation2d getGyroAngle() {
        return new Rotation2d(gyro.getCurrentYaw());
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] arr = new SwerveModulePosition[modules.length];
        for (int i = 0; i < arr.length; i++) {
            arr[i] = modules[i].getModulePosition();
        }
        return arr;
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            modules[i].setState(states[i]);
        }
    }

    Rotation2d gyroAngle;

    @Override
    public void periodic() {
        gyroAngle = getGyroAngle();
        poseEstimator.update(gyroAngle, getModulePositions());

        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    public ChassisSpeeds getChassisSpeedsRobotRel() {
        return kinematicsOld.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getChassisSpeedsFieldRel() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(kinematicsOld.toChassisSpeeds(getModuleStates()), getGyroAngle());
    }

    /**
     * Returns the state of every module
     * 
     * 
     * @return Velocity in m/s, angle in Rotation2d
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] res = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            res[i] = modules[i].getState();
        }
        return res;
    }

    public void setYaw(Rotation2d angle) {
        if (angle != null) {
            gyro.setYaw(angle.getDegrees());
            poseEstimator
                    .resetPose(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), gyro.getRotation2d()));
        }
    }

    public void setVelocitiesRotateToAngleOld(ChassisSpeeds speeds, double angle) {
        double angleError = angle - getGyroAngle().getRadians();
        double angleErrorabs = Math.abs(angleError);
        if (angleErrorabs > Math.toRadians(1.5)) {
            speeds.omegaRadiansPerSecond = angleError * 1.5;
        }

        setVelocitiesWithAccel(speeds);
    }

    public void setVelocitiesRotateToTarget(ChassisSpeeds speeds, Pose2d target) {
      Translation2d robotToTarget = target.getTranslation().minus(getPose().getTranslation());
      double angleError = robotToTarget.getAngle().minus(getGyroAngle()).getRadians();
      double angleErrorabs = Math.abs(angleError);
      if (angleErrorabs > Math.toRadians(1.5)) {
          speeds.omegaRadiansPerSecond = angleError * 2;
      }
      setVelocities(speeds);
    }

    PIDController drivePID = new PIDController(2, 0, 0);

    public void goTo(Pose2d pose, double threshold, boolean stopWhenFinished) {

        Translation2d diffVector = pose.getTranslation().minus(getPose().getTranslation());

        double distance = diffVector.getNorm();
        if (distance <= threshold) {
            if (stopWhenFinished)
                setVelocitiesRotateToAngleOld(new ChassisSpeeds(0, 0, 0), pose.getRotation().getRadians());
            else
                setVelocitiesRotateToAngleOld(
                        new ChassisSpeeds(0.5 * diffVector.getAngle().getCos(), 0.5 * diffVector.getAngle().getSin(),
                                0),
                        pose.getRotation().getRadians());
        }

        else {
            double vX = MathUtil.clamp(-drivePID.calculate(diffVector.getX(), 0), -3.2, 3.2);
            double vY = MathUtil.clamp(-drivePID.calculate(diffVector.getY(), 0), -3.2, 3.2);


            setVelocitiesRotateToAngleOld(new ChassisSpeeds(vX, vY, 0), pose.getRotation().getRadians());
        }

    }

    public void stop() {
        for (SwerveModule i : modules) {
            i.stop();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

    public Trajectory vector(Translation2d start, Translation2d end){
      return TrajectoryGenerator.generateTrajectory(
            List.of(
              new Pose2d(start, end.getAngle().minus(start.getAngle())),
              new Pose2d(end, end.getAngle().minus(start.getAngle()))),
            new TrajectoryConfig(4.0, 4.0));
    }
}
