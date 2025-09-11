package frc.robot.Drive;


import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Demacia.utils.DriverUtils;
import frc.Demacia.utils.Utilities;
import frc.Demacia.utils.Motors.MotorCommands;
import frc.Demacia.utils.Motors.MotorInterface;
import frc.Demacia.utils.Log.SwerveLogEntry;
import frc.Demacia.utils.DriverUtils.JoystickSide;

public class DriveSubsystem2 extends SubsystemBase {

    // components - controller, modules, gyro and motors
    CommandXboxController controller;
    SwerveModule[] modules;                     
    Pigeon2 gyro;
    MotorInterface[] steerMotors;
    MotorInterface[] driveMotors;

    // modules state and data
    String moduleNames[];
    SwerveModulePosition[] modulePositions;
    SwerveModuleState[] moduleStates;
    frc.robot.Drive.SwerveModuleState[] moduleStatePos;

    // pose data
    PoseEstimator poseEstimator;     
    Field2d robotField;
    UdiKinematics1 kinematics;
    Pose2d pose;

    // gyro 
    StatusSignal<Angle> gyroSignal;
    Rotation2d gyroRotation = new Rotation2d();

    // current and target speeds
    ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();
    ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();

    public DriveSubsystem2(CommandXboxController controller) {
        super();
        this.controller = controller;
        // create the arrays of modules and modules data
        modules = new SwerveModule[Constants.CONFIGS.length];
        Translation2d[] modulePositionOnRobot = new Translation2d[modules.length];
        modulePositions = new SwerveModulePosition[modules.length];
        moduleStates = new SwerveModuleState[modules.length];
        moduleStatePos = new frc.robot.Drive.SwerveModuleState[modules.length];
        steerMotors = new MotorInterface[modules.length];
        driveMotors = new MotorInterface[modules.length];
        moduleNames = new String[modules.length];
        // fill modules data
        for(int i = 0; i < modules.length; i++) {
            modules[i] = new SwerveModule(Constants.CONFIGS[i]);
            modulePositionOnRobot[i] = modules[i].config.positionRelativeToRobotCenter;
            moduleStates[i] = modules[i].state;
            modulePositions[i] = modules[i].position;
            steerMotors[i] = modules[i].steerMotor();
            driveMotors[i] = modules[i].driveMotor();
            moduleNames[i] = Constants.CONFIGS[i].name;
            moduleStatePos[i] = modules[i].state;
        }
        // kinemtics and gyro
        kinematics = new UdiKinematics1(modulePositionOnRobot);
        gyro = new Pigeon2(Constants.GYRO_ID, Constants.GYRO_CANBUS.canbus);
        gyroSignal = gyro.getYaw();
        refreshGyro();
        poseEstimator = new PoseEstimator(currentChassisSpeeds, modules[0].steerHeadingSignal(), gyroSignal,new Pose2d());
        pose = new Pose2d();
        robotField = new Field2d();
        updatePose();

        // smart dashboard
        SmartDashboard.putData("Drive", this);
        SmartDashboard.putData("Robot Position", robotField);
        SmartDashboard.putData("Set Drive Brake", new InstantCommand(()-> {for(SwerveModule m : modules) m.setBrake();}).ignoringDisable(true));
        SmartDashboard.putData("Set Drive Coast", new InstantCommand(()-> {for(SwerveModule m : modules) m.setCoast();}).ignoringDisable(true));
        SmartDashboard.putData("Reset Heading", new InstantCommand(this::setFieldHeading).ignoringDisable(true));
        // commands
        controller.start().onTrue(new InstantCommand(this::setFieldHeading).ignoringDisable(true));
        SmartDashboard.putData("Drive Command", new RunCommand(this::drive, this));
//        setDefaultCommand(new RunCommand(this::drive, this));
        showBaseCommands();
        // Log
        SwerveLogEntry.add(moduleNames, moduleStates, modulePositions, pose, currentChassisSpeeds, targetChassisSpeeds);
    }

    /**
     * Show the base Sysid commands
     */
    private void showBaseCommands() {
        MotorCommands.showRandomPowerCommand("Steers Random Power", -6, 6, 0.3, this, steerMotors);
        MotorCommands.showRandomPowerCommand("Drives Random Power", -9, 9, 0.2, this, driveMotors);
        MotorCommands.showPowerCommand("Drives Power", this, driveMotors);
        MotorCommands.showSlowPowerCommand("Steers Slow Power", 0.05, 0.01, 1, this, steerMotors);
        MotorCommands.showMotionCommand("Set Steer Angle",this, steerMotors);
        MotorCommands.showVelocityCommand("Set Drive Velocity",this, driveMotors);

    }

    /**
     * the drive by controller function
     */
    private void drive() {
        targetChassisSpeeds.vxMetersPerSecond = DriverUtils.getJSvalue(controller, JoystickSide.RightY) * Constants.MAX_SPEED;
        targetChassisSpeeds.vyMetersPerSecond = -DriverUtils.getJSvalue(controller, JoystickSide.RightX) * Constants.MAX_SPEED;
        targetChassisSpeeds.omegaRadiansPerSecond = DriverUtils.getTriggerValue(controller) * Constants.MAX_OMEGA;
        setSpeeds(targetChassisSpeeds);
    }

    /**
     * Vision Data update
     * @param pose
     * @param time
     */
    public void updateVisionPosition(Pose2d pose, double time) {
        poseEstimator.updateVision(pose, time);
    }

    /**
     * reset heading to zero
     */
    public void setFieldHeading() {
        resetPose(pose.getTranslation(), new Rotation2d());
        System.out.println(" reset heading");
    }

    /**
     * Gyro data
     * @return
     */
    public void refreshGyro() {
        gyroSignal.refresh();
        gyroRotation.set(gyroSignal.getValue().in(Radians));
    }

    public double getGyroHeading() {
        return gyroRotation.getDegrees();
    }

    /*
     * Heading
     */
    public Rotation2d getHeadingRotation() {
        return pose.getRotation();
    }

    public double getHeading() {
        return getHeadingRotation().getDegrees();
    }

    private edu.wpi.first.math.geometry.Pose2d getPoseCopy() {
        return new edu.wpi.first.math.geometry.Pose2d(
            new edu.wpi.first.math.geometry.Translation2d(pose.getX(), pose.getY()),
            new edu.wpi.first.math.geometry.Rotation2d(pose.getRotation().getRadians()));

    }

    /**
     * reset the pose
     * @param translation2d
     * @param rotation2d
     */
    public void resetPose(Translation2d translation2d, Rotation2d rotation2d) {
        pose.getTranslation().set(translation2d.getX(), translation2d.getY());
        pose.getRotation().set(rotation2d.getRadians());
        poseEstimator.setPose();
        updatePose();
        System.out.println("Reset pose to " + translation2d + " " + rotation2d);
        System.out.println(" new pose - " + pose);
        System.out.println(" heading = " + getHeading());
    }



    private void updatePose() {
        poseEstimator.updatePose();
        robotField.setRobotPose(getPoseCopy());
    }


    /**
     * Set the robot to the required ChassisSpeeds
     * @param speeds
     */
    public void setSpeeds(ChassisSpeeds speeds) {
        if(isZeroZpeed(speeds)) {
            for(SwerveModule m : modules) {
                m.stop();
            }
            return;
        }
        limitSpeeds(speeds);
        var states = kinematics.getModuleStates(getHeading(), speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_SPEED);
        for(int i = 0; i < modules.length; i++) {
            modules[i].setState(states[i]);
        }
    }

    private boolean isZeroZpeed(ChassisSpeeds speeds) {
        return Math.abs(speeds.vxMetersPerSecond) < 0.01 && 
            Math.abs(speeds.vyMetersPerSecond) < 0.01 && 
            Math.abs(speeds.omegaRadiansPerSecond) < 0.01; 
    }

    /**
     * Limit the robot relative speeds based on max acceleration
     * @param speeds
     */
    private void limitSpeeds(ChassisSpeeds speeds) {
        // limit robot relative speeds to account for MAX accelration
        if(Math.abs(speeds.vxMetersPerSecond) > 0.1) {
            double currentX = currentChassisSpeeds.vxMetersPerSecond;
            double newX = currentX + Utilities.clamp(speeds.vxMetersPerSecond-currentX, Constants.MAX_X_VELOCITY_CHANGE);
            double ratio = Math.abs(newX / speeds.vxMetersPerSecond);
            speeds.vxMetersPerSecond *= ratio;
            speeds.vyMetersPerSecond *= ratio;
        } 
        if(Math.abs(speeds.vyMetersPerSecond)  > 0.1) {
            double currentY = currentChassisSpeeds.vyMetersPerSecond;
            double newY = currentY + Utilities.clamp(speeds.vyMetersPerSecond-currentY, Constants.MAX_Y_VELOCITY_CHANGE);
            double ratio = Math.abs(newY / speeds.vxMetersPerSecond);
            speeds.vxMetersPerSecond *= ratio;
            speeds.vyMetersPerSecond *= ratio;
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        for(SwerveModule m : modules) {
            m.refreshStateAndPosition();
        }
        refreshGyro();
        kinematics.getChassisSpeeds(moduleStatePos, getHeading());
        updatePose();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Heading", this::getHeading, null);
        builder.addDoubleProperty("Gyro", this::getGyroHeading, null);
        builder.addDoubleProperty("Vx", ()->currentChassisSpeeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Vy", ()->currentChassisSpeeds.vyMetersPerSecond, null);
        builder.addDoubleProperty("Omega Rad Per Sec", ()->currentChassisSpeeds.omegaRadiansPerSecond, null);
    }

}
