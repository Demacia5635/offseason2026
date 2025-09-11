package frc.Demacia.utils.Motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.Demacia.utils.StatusSignalData;
import frc.Demacia.utils.Elastic.UpdateArray;
import frc.Demacia.utils.Log.LogManager;
import frc.Demacia.utils.Log.MotorLogEntry;

public class TalonMotor extends TalonFX implements MotorInterface {

    TalonConfig config;
    String name;
    TalonFXConfiguration cfg;

    double unitMultiplier = 1.0;
    int slot = 0;

    DutyCycleOut dutyCycle = new DutyCycleOut(0);
    VoltageOut voltageOut = new VoltageOut(0);
    VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(slot);
    MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(slot);
    MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(0).withSlot(slot);
    PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(slot);

    StatusSignalData<ControlModeValue> controlModeSignal;
    StatusSignalData<Double> closedLoopSPSignal;
    StatusSignalData<Double> closedLoopErrorSignal;
    StatusSignalData<Angle> positionSignal;
    StatusSignalData<AngularVelocity> velocitySignal;
    StatusSignalData<AngularAcceleration> accelerationSignal;
    StatusSignalData<Voltage> voltageSignal;
    StatusSignalData<Current> currentSignal;

    String lastControlMode;

    public TalonMotor(TalonConfig config) {
        super(config.id, config.canbus.canbus);
        this.config = config;
        name = config.name;
        configMotor();
        setSignals();
        addLog();
        LogManager.log(name + " motor initialized");
        SmartDashboard.putData(name,this);
    }

    private void configMotor() {
        cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.SupplyCurrentLimit = config.maxCurrent;
        cfg.CurrentLimits.SupplyCurrentLowerLimit = config.maxCurrent;
        cfg.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.rampUpTime;
        cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = config.rampUpTime;

        cfg.MotorOutput.Inverted = config.inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = config.brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        cfg.MotorOutput.PeakForwardDutyCycle = config.maxVolt / 12.0;
        cfg.MotorOutput.PeakReverseDutyCycle = config.minVolt / 12.0;
        if(config.motorRatio < 0.2) {
            unitMultiplier = 100.0;
        }
        cfg.Feedback.SensorToMechanismRatio = config.motorRatio * unitMultiplier;
        updatePID(false);
        cfg.Voltage.PeakForwardVoltage = config.maxVolt;
        cfg.Voltage.PeakReverseVoltage = config.minVolt;
        configureMotionMagic(false);


        getConfigurator().apply(cfg);
    }

    private void configureMotionMagic(boolean apply) {
        cfg.MotionMagic.MotionMagicAcceleration = config.maxAcceleration / unitMultiplier;
        cfg.MotionMagic.MotionMagicCruiseVelocity = config.maxVelocity / unitMultiplier;
        cfg.MotionMagic.MotionMagicJerk = config.maxJerk / unitMultiplier;
        if(config.maxAcceleration > 0) {
            cfg.MotionMagic.MotionMagicExpo_kA = 12.0 / config.maxAcceleration * unitMultiplier;
        } else {
            cfg.MotionMagic.MotionMagicExpo_kA = config.pid[slot].ka() * unitMultiplier;
        }
        if(config.maxVelocity > 0) {
            cfg.MotionMagic.MotionMagicExpo_kV = 12.0 / config.maxVelocity  * unitMultiplier;
        } else {
            cfg.MotionMagic.MotionMagicExpo_kA = config.pid[slot].kv() * unitMultiplier;
        }
        if(apply) {
            getConfigurator().apply(cfg.MotionMagic);
            System.out.println(" motion param " + config.maxVelocity + " , " + config.maxAcceleration + " k=" 
                + cfg.MotionMagic.MotionMagicExpo_kV + ", " + cfg.MotionMagic.MotionMagicExpo_kA);
        }

    }

    private void updatePID(boolean apply) {
        System.out.println("update PID");
        cfg.Slot0.kP = config.pid[0].kp() * unitMultiplier;
        cfg.Slot0.kI = config.pid[0].ki() * unitMultiplier;
        cfg.Slot0.kD = config.pid[0].kd() * unitMultiplier;
        cfg.Slot0.kS = config.pid[0].ks();
        cfg.Slot0.kV = config.pid[0].kv() * unitMultiplier;
        cfg.Slot0.kA = config.pid[0].ka() * unitMultiplier;
        cfg.Slot0.kG = config.pid[0].kg();
        cfg.Slot1.kP = config.pid[1].kp() * unitMultiplier;
        cfg.Slot1.kI = config.pid[1].ki() * unitMultiplier;
        cfg.Slot1.kD = config.pid[1].kd() * unitMultiplier;
        cfg.Slot1.kS = -config.pid[1].ks();
        cfg.Slot1.kV = config.pid[1].kv() * unitMultiplier;
        cfg.Slot1.kA = config.pid[1].ka() * unitMultiplier;
        cfg.Slot1.kG = config.pid[1].kg();
        cfg.Slot2.kP = config.pid[2].kp() * unitMultiplier;
        cfg.Slot2.kI = config.pid[2].ki() * unitMultiplier;
        cfg.Slot2.kD = config.pid[2].kd() * unitMultiplier;
        cfg.Slot2.kS = config.pid[2].ks();
        cfg.Slot2.kV = config.pid[2].kv() * unitMultiplier;
        cfg.Slot2.kA = config.pid[2].ka() * unitMultiplier;
        cfg.Slot2.kG = config.pid[2].kg();
        cfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        cfg.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        cfg.Slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        if(apply) {
            getConfigurator().apply(cfg.Slot0);
            getConfigurator().apply(cfg.Slot1);
            getConfigurator().apply(cfg.Slot2);
        }
    }

    private void setSignals() {
        controlModeSignal = new StatusSignalData<>(getControlMode());
        closedLoopSPSignal = new StatusSignalData<>(getClosedLoopReference(), unitMultiplier);
        closedLoopErrorSignal = new StatusSignalData<>(getClosedLoopError(), unitMultiplier);
        positionSignal = new StatusSignalData<>(getPosition(), unitMultiplier);
        velocitySignal = new StatusSignalData<>(getVelocity(), unitMultiplier);
        accelerationSignal = new StatusSignalData<>(getAcceleration(), unitMultiplier);
        voltageSignal = new StatusSignalData<>(getMotorVoltage());
        currentSignal = new StatusSignalData<>(getStatorCurrent());
    }

    private void addLog() {
        MotorLogEntry.add(this);
    }

    public void checkElectronics() {
        if (getFaultField().getValue() != 0) {
            LogManager.log(name + " have fault num: " + getFaultField().getValue(), AlertType.kError);
        }
    }

    /**
     * change the slot of the pid and feed forward.
     * will not work if the slot is null
     * 
     * @param slot the wanted slot between 0 and 2
     */
    public void changeSlot(int slot) {
        if (slot < 0 || slot > 2) {
            LogManager.log("slot is not between 0 and 2", AlertType.kError);
            return;
        }
        this.slot = slot;
        velocityVoltage.withSlot(slot);
        motionMagicVoltage.withSlot(slot);
        motionMagicExpoVoltage.withSlot(slot);
        positionVoltage.withSlot(slot);
    }

    /*
     * set motor to brake or coast
     */
    public void setNeutralMode(boolean isBrake) {
        cfg.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        getConfigurator().apply(cfg.MotorOutput);
    }

    /**
     * set power from 1 to -1 (v/12) no PID/FF
     * 
     * @param power the wanted power between -1 to 1
     */
    public void setDuty(double power) {
        setControl(dutyCycle.withOutput(power));
        // dutyCycleEntry.log(power);
    }

    public void setVoltage(double voltage) {
        setControl(voltageOut.withOutput(voltage));
        // dutyCycleEntry.log(voltage / 12.0);
    }

    /**
     * set volocity to motor with PID and FF
     * 
     * @param velocity    the wanted velocity in meter per second or radians per
     *                    seconds depending on the config
     * @param feedForward wanted feed forward to add to the ks kv ka and kg,
     *                    defaults to 0
     */
    public void setVelocity(double velocity, double feedForward) {
        setControl(velocityVoltage.withVelocity(velocity/unitMultiplier).withFeedForward(feedForward));
        // velocityEntry.log(velocity);
    }

    public void setVelocity(double velocity) {
        setVelocity(velocity, 0);
    }

    /**
     * set motion magic with PID and Ff
     * <br>
     * </br>
     * must add to config motion magic configs (vel, acc, jerk[optional])
     * 
     * @param position    the wanted position in meter or radians depending on the
     *                    config
     * @param feedForward wanted feed forward to add to the ks kv ka and kg defaults
     *                    to 0
     */
    public void setMotion(double position, double feedForward) {
        setControl(motionMagicExpoVoltage.withPosition(position/unitMultiplier).withFeedForward(feedForward));  
        /* 
        double error = position-getCurrentPosition(); 
        if(error > 0) {
            setControl(motionMagicExpoVoltage.withPosition(position/unitMultiplier).withFeedForward(feedForward).withSlot(0));  
        } else {
            setControl(motionMagicExpoVoltage.withPosition(position/unitMultiplier).withFeedForward(feedForward).withSlot(1));  
        }
            */
    }

    public void setMotion(double position) {
        setMotion(position, 0);
    }
    @Override
    public void setAngle(double angle, double feedForward) {
      setMotion(MotorUtils.getPositionForAngle(getCurrentPosition(), angle, config.isRadiansMotor), feedForward);
    }
    @Override
    public void setAngle(double angle) {
      setMotion(MotorUtils.getPositionForAngle(getCurrentPosition(), angle, config.isRadiansMotor));
    }
  

    public void setPositionVoltage(double position, double feedForward) {
        setControl(positionVoltage.withPosition(position/unitMultiplier).withFeedForward(feedForward));
        // positionEntry.log(position);
    }

    public void setPositionVoltage(double position) {
        setPositionVoltage(position, 0);
    }

    public void setVelocityWithFeedForward(double velocity) {
        setVelocity(velocity, velocityFeedForward(velocity));
    }

    public void setMotionWithFeedForward(double velocity) {
        setVelocity(velocity, positionFeedForward(velocity));
    }

    private double velocityFeedForward(double velocity) {
        return velocity * velocity * Math.signum(velocity) * config.kv2;
    }

    private double positionFeedForward(double positin) {
        return Math.cos(positin * config.posToRad) * config.kSin;
    }

    public String getCurrentControlMode() {
        return controlModeSignal.getString();
    }

    public double getCurrentClosedLoopSP() {
        return closedLoopSPSignal.get();
    }

    public double getCurrentClosedLoopError() {
        return closedLoopErrorSignal.get();
    }

    public double getCurrentPosition() {
        return positionSignal.get();
    }
    public double getCurrentAngle() {
        if(config.isRadiansMotor) {
            return MathUtil.angleModulus(getCurrentPosition());
        } else if(config.isDegreesMotor) {
            return MathUtil.inputModulus(getCurrentPosition(), -180, 180);
        }
        return 0;
    }

    public double getCurrentVelocity() {
        return velocitySignal.get();
    }

    public double getCurrentAcceleration() {
        return accelerationSignal.get();
    }

    public double getCurrentVoltage() {
        return voltageSignal.get();
    }
    public double getCurrentCurrent() {
        return currentSignal.get();
    }

    /**
     * creates a widget in elastic of the pid and ff for hot reload
     * 
     * @param slot the slot of the close loop perams (from 0 to 2)
     */
    public void showConfigPIDFSlotCommand(int slot) {
        CloseLoopParam p = config.pid[slot];
        if(p != null) {
            UpdateArray.show(name + " PID " + slot , CloseLoopParam.names, p.toArray(),(double[] array)->updatePID(true));
        }
    }

    /**
     * creates a widget in elastic to configure motion magic in hot reload
     */
    public void showConfigMotionVelocitiesCommand() {
        UpdateArray.show(name + "MOTION PARAM",
            new String[] {"Velocity", "Acceleration", "Jerk"},
            new double[] {config.maxVelocity, config.maxAcceleration, config.maxJerk},
            (double[] array)->{
                config.maxVelocity = array[0];
                config.maxAcceleration = array[1];
                config.maxJerk = array[2];
                configureMotionMagic(true);
            });
    }

    /**
     * override the sendable of the talonFX to our costum widget in elastic
     * <br>
     * </br>
     * to activate put in the code:
     * 
     * <pre>
     * SmartDashboard.putData("talonMotor name", talonMotor);
     * </pre>
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Talon Motor");
        builder.addDoubleProperty("CloseLoopError", this::getCurrentClosedLoopError, null);
        builder.addDoubleProperty("Position", this::getCurrentPosition, null);
        builder.addDoubleProperty("Velocity", this::getCurrentVelocity, null);
        builder.addDoubleProperty("Acceleration", this::getCurrentAcceleration, null);
        builder.addDoubleProperty("Voltage", this::getCurrentVoltage, null);
        builder.addDoubleProperty("Current", this::getCurrentCurrent, null);
        if(config.isDegreesMotor || config.isRadiansMotor) {
            builder.addDoubleProperty("Angle", this::getCurrentAngle, null);
        }
    }

    public double gearRatio() {
        return config.motorRatio;
    }

    public String name() {
        return name;
    }

    @Override
    public void setEncoderPosition(double position) {
      setPosition(position / unitMultiplier);   
    }
    public StatusSignalData<Double> getClosedLoopErrorSignal() {
        return closedLoopErrorSignal;
    }
    public StatusSignalData<Double> getClosedLoopSPSignal() {
        return closedLoopSPSignal;
    }
    public StatusSignalData<Angle> getPositionSignal() {
        return positionSignal;
    }
    public StatusSignalData<AngularVelocity> getVelocitySignal() {
        return velocitySignal;
    }
    public StatusSignalData<AngularAcceleration> getAccelerationSignal() {
        return accelerationSignal;
    }
    public StatusSignalData<Voltage> getVoltageSignal() {
        return voltageSignal;
    }
    public StatusSignalData<Current> getCurrentSignal() {
        return currentSignal;
    }
  @Override
  public void showSysidCommands(Subsystem subsystem) {
        MotorUtils.showSysidCommands(this, config, subsystem);
    }

}