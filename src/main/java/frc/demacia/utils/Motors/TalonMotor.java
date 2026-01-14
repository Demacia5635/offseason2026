package frc.demacia.utils.Motors;

import com.ctre.phoenix6.StatusSignal;
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
import frc.demacia.utils.Data;
import frc.demacia.utils.UpdateArray;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;

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

    Data<ControlModeValue> controlModeSignal;
    Data<Double> closedLoopSPSignal;
    Data<Double> closedLoopErrorSignal;
    Data<Angle> positionSignal;
    Data<AngularVelocity> velocitySignal;
    Data<AngularAcceleration> accelerationSignal;
    Data<Voltage> voltageSignal;
    Data<Current> currentSignal;

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
            cfg.MotionMagic.MotionMagicExpo_kV = config.pid[slot].kv() * unitMultiplier;
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
        cfg.Slot1.kS = config.pid[1].ks();
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

    @SuppressWarnings("unchecked")
    private void setSignals() {
        controlModeSignal = new Data<>(getControlMode());
        closedLoopSPSignal = new Data<>(getClosedLoopReference());
        closedLoopErrorSignal = new Data<>(getClosedLoopError());
        positionSignal = new Data<>(getPosition());
        velocitySignal = new Data<>(getVelocity());
        accelerationSignal = new Data<>(getAcceleration());
        voltageSignal = new Data<>(getMotorVoltage());
        currentSignal = new Data<>(getStatorCurrent());
    }

    @SuppressWarnings("unchecked")
    private void addLog() {
        LogManager.addEntry(name + "/Position and Velocity and Acceleration and Voltage and Current and CloseLoopError and CloseLoopSP",  new StatusSignal[] {
            positionSignal.getSignal(),
            velocitySignal.getSignal(),
            accelerationSignal.getSignal(),
            voltageSignal.getSignal(),
            currentSignal.getSignal(),
            closedLoopErrorSignal.getSignal(),
            closedLoopSPSignal.getSignal(),
            }).withLogLevel(LogLevel.LOG_ONLY_NOT_IN_COMP)
            .WithIsMotor().build();
        LogManager.addEntry(name + "/ControlMode", 
            controlModeSignal.getSignal())
            .withLogLevel(LogLevel.LOG_ONLY_NOT_IN_COMP).build();
    }

    public void checkElectronics() {
        int fault = getFaultField().getValue();
        if (fault != 0) {
            LogManager.log(name + " have fault num: " + fault, AlertType.kError);
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
    }

    public void setVoltage(double voltage) {
        setControl(voltageOut.withOutput(voltage));
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
    }

    public void setPositionVoltage(double position) {
        setPositionVoltage(position, 0);
    }

    public void setVelocityWithFeedForward(double velocity) {
        setVelocity(velocity, velocityFeedForward(velocity));
    }

    public void setMotionWithFeedForward(double velocity) {
        setMotion(velocity, positionFeedForward(velocity));
    }

    private double velocityFeedForward(double velocity) {
        return velocity * velocity * Math.signum(velocity) * config.kv2;
    }

    private double positionFeedForward(double position) {
        return Math.cos(position * config.posToRad) * config.kSin;
    }

    public String getCurrentControlMode() {
        return controlModeSignal.getString();
    }

    public double getCurrentClosedLoopSP() {
        Double value = closedLoopSPSignal.getDouble();
        return value != null ? value * unitMultiplier : 0.0;
    }
    
    public double getCurrentClosedLoopError() {
        Double value = closedLoopErrorSignal.getDouble();
        return value != null ? value * unitMultiplier : 0.0;
    }
    
    public double getCurrentPosition() {
        Double value = positionSignal.getDouble();
        return value != null ? value * unitMultiplier : 0.0;
    }
    
    public double getCurrentVelocity() {
        Double value = velocitySignal.getDouble();
        return value != null ? value * unitMultiplier : 0.0;
    }
    
    public double getCurrentAcceleration() {
        Double value = accelerationSignal.getDouble();
        return value != null ? value * unitMultiplier : 0.0;
    }
    
    public double getCurrentAngle() {
        if(config.isRadiansMotor) {
            return MathUtil.angleModulus(getCurrentPosition());
        } else if(config.isDegreesMotor) {
            return MathUtil.inputModulus(getCurrentPosition(), -180, 180);
        }
        return 0;
    }
    
    public double getCurrentVoltage() {
        Double value = voltageSignal.getDouble();
        return value != null ? value : 0.0;
    }
    
    public double getCurrentCurrent() {
        Double value = currentSignal.getDouble();
        return value != null ? value : 0.0;
    }

    /**
     * creates a widget in elastic of the pid and ff for hot reload
     * 
     * @param slot the slot of the close loop perams (from 0 to 2)
     */
    public void showConfigPIDFSlotCommand(int slot) {
        CloseLoopParam p = config.pid[slot];
        if(p != null) {
            UpdateArray.show(name + " PID " + slot , CloseLoopParam.PARAMETER_NAMES, p.toArray(),(double[] array)->updatePID(true));
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

    public void showConfigMotorCommand() {
        UpdateArray.show(name + " MOTOR CONFIG",
            new String[] {
                "Max Current",
                "Ramp Time (s)",
                "Max Volt",
                "Brake (0/1)",
                "Invert (0/1)",
                "Motor Ratio",
                "Slot"
            },
            new double[] {
                config.maxCurrent,
                config.rampUpTime,
                config.maxVolt,
                config.brake ? 1.0 : 0.0,
                config.inverted ? 1.0 : 0.0,
                config.motorRatio,
                slot
            },
            (double[] array) -> {
                config.withCurrent(array[0])
                      .withRampTime(array[1])
                      .withVolts(array[2])
                      .withBrake(array[3] > 0.5)
                      .withInvert(array[4] > 0.5);
    
                config.motorRatio = array[5];
    
                configMotor();
                changeSlot(slot);
    
                System.out.println("[HOT RELOAD] Motor config updated for " + name);
            }
        );
    }

    public void showControlCommand() {
        UpdateArray.show(name + " CONTROL",
            new String[] {
                "ControlMode (0=Duty, 1=Voltage, 2=Velocity, 3=MotionMagic, 4=angle, 5=positionVoltage, 6=velocityWithFeedForward, 7=motionWithFeedForward)",
                "Value"
            },
            new double[] {
                0, // default control mode: Duty
                0  // default value
            },
            (double[] array) -> {
                int mode = (int) array[0];
                double value = array[1];
    
                switch (mode) {
                    case 0: // Duty cycle [-1, 1]
                        setDuty(value);
                        break;
                    case 1: // Voltage
                        setVoltage(value);
                        break;
                    case 2: // Velocity
                        setVelocity(value);
                        break;
                    case 3: // MotionMagic
                        setMotion(value);
                        break;
                    case 4: // angle
                        setAngle(value);
                        break;
                    case 5: // positionVoltage
                        setPositionVoltage(value);
                        break;
                    case 6: // velocityWithFeedForward
                        setVelocityWithFeedForward(value);
                        break;
                    case 7: // MotionMagic
                        setMotionWithFeedForward(value);
                        break;
                    default:
                        System.out.println("[CONTROL] Invalid mode: " + mode);
                }
            }
        );
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
        builder.addStringProperty("ControlMode", this::getCurrentControlMode, null);
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
    public Data<Double> getClosedLoopErrorSignal() {
        return closedLoopErrorSignal;
    }
    public Data<Double> getClosedLoopSPSignal() {
        return closedLoopSPSignal;
    }
    public Data<Angle> getPositionSignal() {
        return positionSignal;
    }
    public Data<AngularVelocity> getVelocitySignal() {
        return velocitySignal;
    }
    public Data<AngularAcceleration> getAccelerationSignal() {
        return accelerationSignal;
    }
    public Data<Voltage> getVoltageSignal() {
        return voltageSignal;
    }
    public Data<Current> getCurrentSignal() {
        return currentSignal;
    }
}