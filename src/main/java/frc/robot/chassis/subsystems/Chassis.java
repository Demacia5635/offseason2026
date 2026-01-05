package frc.robot.chassis.subsystems;

import java.util.List;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Log.LogManager;
import frc.robot.RobotContainer;
import static frc.robot.chassis.utils.ChassisConstants.*;

import frc.robot.chassis.utils.ChassisConstants.AccelConstants;
import frc.robot.kinematics.DemaciaKinematics;

public class Chassis extends SubsystemBase {
    private SwerveModule[] modules;
    private Pigeon2 gyro;

    private final DemaciaKinematics demaciaKinematics;
    private final SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field;

    private StatusSignal<Angle> gyroYawStatus;
    private Rotation2d lastGyroYaw;

    public Chassis() {
        modules = new SwerveModule[] {
                new SwerveModule(FRONT_LEFT),
                new SwerveModule(FRONT_RIGHT),
                new SwerveModule(BACK_LEFT),
                new SwerveModule(BACK_RIGHT),
        };
        gyro = new Pigeon2(GYRO_ID, GYRO_CAN_BUS);
        addStatus();
        kinematics = new SwerveDriveKinematics(
                FRONT_LEFT.POSITION,
                FRONT_RIGHT.POSITION,
                BACK_LEFT.POSITION,
                BACK_RIGHT.POSITION

        );
        demaciaKinematics = new DemaciaKinematics(new Translation2d[] {
                FRONT_LEFT.POSITION,
                FRONT_RIGHT.POSITION,
                BACK_LEFT.POSITION,
                BACK_RIGHT.POSITION });

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroAngle(), getModulePositions(), new Pose2d());

        SimpleMatrix std = new SimpleMatrix(new double[] { 0.02, 0.02, 0 });
        poseEstimator.setVisionMeasurementStdDevs(new Matrix<>(std));
        field = new Field2d();

        SmartDashboard.putData("chassis", this);
        LogManager.addEntry("chassis/vx robot rel", () -> getChassisSpeedsRobotRel().vxMetersPerSecond);
        LogManager.addEntry("chassis/vy robot rel", () -> getChassisSpeedsRobotRel().vyMetersPerSecond);
        SmartDashboard.putData("reset gyro", new InstantCommand(() -> setYaw(Rotation2d.kZero)).ignoringDisable(true));
        SmartDashboard.putData("reset gyro 180",
                new InstantCommand(() -> setYaw(Rotation2d.kPi)).ignoringDisable(true));
        SmartDashboard.putData("chassis/set coast",
                new InstantCommand(() -> setNeutralMode(false)).ignoringDisable(true));
        SmartDashboard.putData("chassis/set brake",
                new InstantCommand(() -> setNeutralMode(true)).ignoringDisable(true));
        // SmartDashboard.putData(getName() + "/Swerve Drive", getChassisWidget());
        // SmartDashboard.putData("Chassis", this);
    }

    // private Sendable getChassisWidget() {
    // return new Sendable() {
    // @Override
    // public void initSendable(SendableBuilder builder) {
    // builder.setSmartDashboardType("SwerveDrive");

    // builder.addDoubleProperty("Front Left Angle", () ->
    // modules[0].getAbsoluteAngle(), null);
    // builder.addDoubleProperty("Front Left Velocity", () ->
    // modules[0].getDriveVel(), null);

    // builder.addDoubleProperty("Front Right Angle", () ->
    // modules[1].getAbsoluteAngle(), null);
    // builder.addDoubleProperty("Front Right Velocity", () ->
    // modules[1].getDriveVel(), null);

    // builder.addDoubleProperty("Back Left Angle", () ->
    // modules[2].getAbsoluteAngle(), null);
    // builder.addDoubleProperty("Back Left Velocity", () ->
    // modules[2].getDriveVel(), null);

    // builder.addDoubleProperty("Back Right Angle", () ->
    // modules[3].getAbsoluteAngle(), null);
    // builder.addDoubleProperty("Back Right Velocity", () ->
    // modules[3].getDriveVel(), null);

    // builder.addDoubleProperty("Robot Angle", () -> getGyroAngle().getRadians(),
    // null);
    // }
    // };
    // }

    public void setNeutralMode(boolean isBrake) {
        for (SwerveModule module : modules) {
            module.setNeutralMode(isBrake);
        }
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    private void addStatus() {
        gyroYawStatus = gyro.getYaw();
        lastGyroYaw = new Rotation2d(gyroYawStatus.getValue());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setVelocities(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroAngle());
        // speeds = ChassisSpeeds.discretize(speeds, CYCLE_DT);
        SwerveModuleState[] states = demaciaKinematics.udiTest(speeds, getChassisSpeedsRobotRel());// , getChassisSpeedsRobotRel());
        if(Math.abs(getModuleStates()[0].speedMetersPerSecond) > 0.2)LogManager.log("real state: " + getModuleStates()[0]);
        setModuleStates(states);
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

    public double getSteeracceleration(int id) {
        return modules[id].getSteerAccel();
    }

    public void setSteerPositions(double position) {
        setSteerPositions(new double[] { position, position, position, position });
    }

    public ChassisSpeeds getRobotRelVelocities() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(getChassisSpeedsRobotRel(), getGyroAngle());
    }

    public void setDriveVelocities(double[] velocities) {
        for (int i = 0; i < velocities.length; i++) {
            modules[i].setDriveVelocity(velocities[i]);
        }
    }

    public void setDriveVelocities(double velocity) {
        setDriveVelocities(new double[] { velocity, velocity, velocity, velocity });
    }

    public boolean isRed() {
        return RobotContainer.isRed();
    }

    public Rotation2d getGyroAngle() {
        gyroYawStatus.refresh();
        if (gyroYawStatus.getStatus() == StatusCode.OK) {
            lastGyroYaw = new Rotation2d(gyroYawStatus.getValue());
        }
        return lastGyroYaw;
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
        return demaciaKinematics.toChassisSpeeds(getModuleStates(),
                Math.toRadians(gyro.getAngularVelocityZWorld().getValueAsDouble()));
    }

    public ChassisSpeeds getChassisSpeedsFieldRel() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeedsRobotRel(), getGyroAngle());
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

    public void stop() {
        for (SwerveModule i : modules) {
            i.stop();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("chassis/vx", () -> getChassisSpeedsRobotRel().vxMetersPerSecond, null);

        builder.addDoubleProperty("chassis/vy", () -> getChassisSpeedsRobotRel().vyMetersPerSecond, null);
    }

    public Trajectory vector(Translation2d start, Translation2d end) {
        return TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(start, end.getAngle().minus(start.getAngle())),
                        new Pose2d(end, end.getAngle().minus(start.getAngle()))),
                new TrajectoryConfig(4.0, 4.0));
    }
}
