package frc.robot.Drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.Demacia.Geometry.Rotation2d;
import frc.Demacia.utils.Motors.MotorCommands;
import frc.Demacia.utils.Motors.MotorInterface;
import frc.Demacia.utils.Motors.TalonMotor;
import frc.Demacia.utils.Sensors.Cancoder;

public class SwerveModule implements Sendable {
    private MotorInterface steer;
    private MotorInterface drive;
    private Cancoder absEncoder;
    protected Constants.ModuleConfig config;
    protected SwerveModulePosition position = new SwerveModulePosition(0, new Rotation2d());
    private double lastSteerPosition = 0;
    private double steerCorrection = 0;
    private double driveTarget = 0;
    protected SwerveModuleState state = new SwerveModuleState(0, 0, 0);


    SwerveModule(Constants.ModuleConfig config) {
        this.config = config;
        steer = new TalonMotor(config.steerConfig);
        drive = new TalonMotor(config.driveConfig);
        absEncoder = new Cancoder(config.cancoderConfig);
        System.out.println("Module " + config.name + " steer=" + config.steerConfig.id);
        setSteerOffset();
        refreshStateAndPosition();
        SmartDashboard.putData(config.name, this);
    }

    public void setSteerOffset() {
        steer.setEncoderPosition(getAbsEncoder()-config.cancoderOffset);
    }

    public double getAbsEncoder() {
        return absEncoder.getCurrentAbsPosition();
    }


    public void refreshStateAndPosition() {
        double steerPosition = steer.getCurrentPosition();
        position.angle.setDegrees((lastSteerPosition + steerPosition)/2);
        lastSteerPosition = steerPosition;
        position.distanceMeters = drive.getCurrentPosition() + steerPosition * Constants.STEER_TO_DISTANCE_RATIO;
        state.angle.setDegrees(steerPosition);
        state.speedMetersPerSecond = drive.getCurrentVelocity() +  steer.getCurrentVelocity() * Constants.STEER_TO_DISTANCE_RATIO;
        state.distanceMeters = position.distanceMeters;
    }

    private void optimaizeTarget() {
        if(steerCorrection > 90) {
            steerCorrection -= 180;
            driveTarget = -driveTarget;
        } else if(steerCorrection < -90) {
            steerCorrection += 180;
            driveTarget = -driveTarget;
        }
        if(Math.abs(steerCorrection) < Constants.MAX_STEER_CORRECTION) {
            steerCorrection *= Constants.STATE_STEER_MULTIPLIER;
        }
    }

    public void setState(edu.wpi.first.math.kinematics.SwerveModuleState state) {
        double currentPosition = steer.getCurrentPosition();
        steerCorrection = MathUtil.inputModulus(state.angle.getDegrees() - currentPosition,-180,180);
        driveTarget = state.speedMetersPerSecond;
        optimaizeTarget();
        steer.setMotion(currentPosition + steerCorrection);
        drive.setVelocity(driveTarget - steer.getCurrentVelocity()*Constants.STEER_TO_DISTANCE_RATIO);
    }

    public void setSteerPower(double power) {
        steer.setDuty(power);
    }
    public void setDrivePower(double power) {
        drive.setDuty(power);
    }

    public void stop() {
        steer.setDuty(0);
        drive.setDuty(0);
    }
    
    public void setSteerAngle(double angle) {
        steer.setMotion(angle);
    }
    public void setDriveVelocity(double velocity) {
        drive.setVelocity(velocity);
    }

    public void showConfigPID() {
        steer.showConfigPIDFSlotCommand(0);
        drive.showConfigPIDFSlotCommand(0);
        steer.showConfigMotionVelocitiesCommand();
        drive.showConfigMotionVelocitiesCommand();
    }

    public void showBaseCommands(Subsystem subsystem) {
        MotorCommands.showRandomPowerCommand(config.name + " Steer Random Power",-0.6, 0.6, 0.2, subsystem, steer);
        MotorCommands.showRandomPowerCommand(config.name + " Drive Random Power",-1, 1, 0.2, subsystem, drive);
        MotorCommands.showSlowPowerCommand(config.name + " Steer Slow Power",0.07, 0.01, 1, subsystem, steer);
        MotorCommands.showSlowPowerCommand(config.name + " Drive Slow Power",0.01, 0.01, 1, subsystem, drive);
        MotorCommands.showMotionCommand(config.name + " Steer Angle",subsystem, steer);
        MotorCommands.showVelocityCommand(config.name + " Drive Velocity",subsystem, drive);
    }

    protected MotorInterface steerMotor() {
        return steer;
    }
    protected MotorInterface driveMotor() {
        return drive;
    }

    public void setBrake() {
        steer.setNeutralMode(true);
        drive.setNeutralMode(true);
    }
    public void setCoast() {
        System.out.println(config.name + " coast");
        steer.setNeutralMode(false);
        drive.setNeutralMode(false);
    }

    public StatusSignal<Angle> steerHeadingSignal() {
        return ((TalonFX)steer).getPosition();
    }



    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("AbsEncoder",this::getAbsEncoder, null);
        builder.addDoubleProperty("distance", ()->state.distanceMeters, null);
        builder.addDoubleProperty("velocity", ()->state.speedMetersPerSecond, null);
        builder.addDoubleProperty("angle", ()->state.angle.getDegrees(), null);
    }
}
