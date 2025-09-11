package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Demacia.utils.Motors.SparkConfig;
import frc.Demacia.utils.Motors.SparkMotor;

import static frc.robot.Constants.ModuleConstants.*;

public class ModuleSubsystem extends SubsystemBase {
    SparkMotor steerMotor;
    SparkMotor driveMotor;
    CANcoder absEncoder;

    SimpleMotorFeedforward steerFF = new SimpleMotorFeedforward(STEER_KS, STEER_KV, 0);
    SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, 0);
    PIDController steerPID = new PIDController(STEER_KP,STEER_KI, STEER_KD);
    PIDController drivePID = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);

    // for simulation
    SparkMaxSim steerSim;
    SparkMaxSim driveSim;
    DCMotorSim steerDCMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.05, STEER_GERA_RATIO * Math.PI), DCMotor.getNEO(1));
    DCMotorSim driveDCMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 2, DRIVE_GERA_RATIO*WHEEL_CIRCUMFERENCE), DCMotor.getNEO(1));

    public ModuleSubsystem() {
        super();
        steerMotor = new SparkMotor(new SparkConfig(STEER_ID,"SteerMotor")
                .withBrake(true)
                .withCurrent(MAX_STEER_AMPS)
                .withInvert(STEER_INVERTED)
                .withRampTime(STEER_RAMP)
                .withVolts(MAX_STEER_VOLTS)
                .withDegreesMotor(STEER_GERA_RATIO)
                .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, STEER_KA, 0));
        driveMotor = new SparkMotor(new SparkConfig(DRIVE_ID,"DriveMotor")
                .withBrake(true)
                .withInvert(DRIVE_INVERTED)
                .withRampTime(DRIVE_RAMP)
                .withMeterMotor(DRIVE_GERA_RATIO,WHEEL_CIRCUMFERENCE)
                .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, 0));
        absEncoder = new CANcoder(CANBCODER_ID);

        // for simulation
        steerSim = new SparkMaxSim(steerMotor, DCMotor.getNEO(1));
        driveSim = new SparkMaxSim(driveMotor, DCMotor.getNEO(1));

        // set steer direction
        calibrateSteer();

        SmartDashboard.putData("Module", this);
    }

    public double getSteerPower() {
        return steerMotor.getCurrentVoltage();
    }
    public double getSteerAngle() {
        return steerMotor.getCurrentPosition();
    }
    public double getSteerVelocity() {
        return steerMotor.getCurrentVelocity();
    }
    public void setSteerPower(double power) {
        steerMotor.setDuty(power);
    }
    public double getDrivePower() {
        return driveMotor.getCurrentVoltage();
    }
    public double getDriveAngle() {
        return driveMotor.getCurrentPosition();
    }
    public double getDriveVelocity() {
        return driveMotor.getCurrentVelocity();
    }
    public void setDrivePower(double power) {
        driveMotor.setDuty(power);
    }

    public double getAbsAngle() {
        return absEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    private void calibrateSteer() {
        double angle = getAbsAngle() - ABS_ENCODER_OFFSET;
        steerMotor.getEncoder().setPosition(angle);
    }

    public void setDriveVelocity(double velocity) {
        double ff = driveFF.calculate(velocity);
        double pid = drivePID.calculate(getDriveVelocity(), velocity);
        setDrivePower(pid + ff);;
    }
    public void setSteerVelocity(double velocity) {
        double ff = steerFF.calculate(velocity);
        double pid = steerPID.calculate(getSteerVelocity(), velocity);
        setSteerPower(pid + ff);;
    }

    public void setSteerAngle(double angle) {
        double velocity = (angle - getSteerAngle()) * STEER_VELOCITY_P;
        setSteerVelocity(velocity);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Abs Angle", this::getAbsAngle, null);

        SmartDashboard.putNumber("Set Steer Power", 0);
        SmartDashboard.putNumber("Set Drive Power", 0);
        SmartDashboard.putData("Set Steer", new RunCommand(()->setSteerPower(SmartDashboard.getNumber("Set Steer Power", 0)), this));
        SmartDashboard.putData("Set Drive", new RunCommand(()->setDrivePower(SmartDashboard.getNumber("Set Drive Power", 0)), this));
        SmartDashboard.putNumber("Set Steer Angle", 0);
        SmartDashboard.putNumber("Set Drive Velocity", 0);
        SmartDashboard.putData("Set Angle", new RunCommand(()->setSteerAngle(SmartDashboard.getNumber("Set Steer Angle", 0)), this));
        SmartDashboard.putData("Set Velocity", new RunCommand(()->setDriveVelocity(SmartDashboard.getNumber("Set Drive Velocity", 0)), this));
        SmartDashboard.putData("Stop", new InstantCommand(()->{setSteerPower(0); setDrivePower(0);}, this));
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
        steerDCMotorSim.setInput(steerSim.getAppliedOutput()*12 );
        driveDCMotorSim.setInput(driveSim.getAppliedOutput()*12);
        steerDCMotorSim.update(0.02);
        driveDCMotorSim.update(0.02);
        steerSim.iterate(steerDCMotorSim.getAngularVelocityRadPerSec(),12, 0.02);
        driveSim.iterate(driveDCMotorSim.getAngularVelocityRadPerSec(),12, 0.02);

    }


}
