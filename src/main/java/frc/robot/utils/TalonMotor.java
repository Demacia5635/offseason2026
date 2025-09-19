package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TalonMotor extends TalonFX {

	TalonConfig config;
  String name;
  TalonFXConfiguration cfg;

  DutyCycleOut dutyCycle = new DutyCycleOut(0);
  VoltageOut voltageOut = new VoltageOut(0);
  VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

  StatusSignal<ControlModeValue> controlModeSignal;
  StatusSignal<Double> closedLoopSPSignal;
  StatusSignal<Double> closedLoopErrorSignal;
  StatusSignal<Angle> positionSignal;
  StatusSignal<AngularVelocity> velocitySignal;
  StatusSignal<AngularAcceleration> accelerationSignal;
  StatusSignal<Voltage> voltageSignal;

  String lastControlMode;
  double lastClosedLoopSP;
  double lastClosedLoopError;
  double lastPosition;
  double lastVelocity;
  double lastAcceleration;
  double lastVoltage;


  public TalonMotor(TalonConfig config) {
		super(config.id, config.canbus);
		this.config = config;
		name = config.name;
		configMotor();
    setSignals();
		addLog();
		LogManager.log(name + " motor initialized");
  }
  
  private void configMotor() {
		cfg = new TalonFXConfiguration();
		cfg.CurrentLimits.SupplyCurrentLimit = config.maxCurrent;
    cfg.CurrentLimits.SupplyCurrentLowerLimit = config.maxCurrent;
    cfg.CurrentLimits.SupplyCurrentLowerTime = config.maxCurrentTriggerTime;
		cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

		cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.rampUpTime;
		cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = config.rampUpTime;

		cfg.MotorOutput.Inverted = config.inverted ? InvertedValue.CounterClockwise_Positive
			: InvertedValue.Clockwise_Positive;
		cfg.MotorOutput.NeutralMode = config.brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		cfg.MotorOutput.PeakForwardDutyCycle = config.maxVolt / 12.0;
		cfg.MotorOutput.PeakReverseDutyCycle = config.minVolt / 12.0;

		cfg.Slot0.kP = config.pid.kp;
		cfg.Slot0.kI = config.pid.ki;
		cfg.Slot0.kD = config.pid.kd;
		cfg.Slot0.kS = config.pid.ks; 
		cfg.Slot0.kV = config.pid.kv;
		cfg.Slot0.kA = config.pid.ka;
		cfg.Slot0.kG = config.pid.kg;
		if(config.pid1 != null) {
			cfg.Slot1.kP = config.pid1.kp;
			cfg.Slot1.kI = config.pid1.ki;
			cfg.Slot1.kD = config.pid1.kd;
			cfg.Slot1.kS = config.pid1.ks; 
			cfg.Slot1.kV = config.pid1.kv;
			cfg.Slot1.kA = config.pid1.ka;
			cfg.Slot1.kG = config.pid1.kg;
		}
		if(config.pid2 != null) {
			cfg.Slot2.kP = config.pid2.kp;
			cfg.Slot2.kI = config.pid2.ki;
			cfg.Slot2.kD = config.pid2.kd;
			cfg.Slot2.kS = config.pid2.ks; 
			cfg.Slot2.kV = config.pid2.kv;
			cfg.Slot2.kA = config.pid2.ka;
			cfg.Slot2.kG = config.pid2.kg;
		}

		cfg.Voltage.PeakForwardVoltage = config.maxVolt;
		cfg.Voltage.PeakReverseVoltage = config.minVolt;

		cfg.Feedback.SensorToMechanismRatio = config.motorRatio;

		cfg.MotionMagic.MotionMagicAcceleration = config.motionMagicAccel;
		cfg.MotionMagic.MotionMagicCruiseVelocity = config.motionMagicVelocity;
		cfg.MotionMagic.MotionMagicJerk = config.motionMagicJerk;

		// dutyCycle.UpdateFreqHz = 200;
    // voltageOut.UpdateFreqHz = 200;
		// velocityVoltage.UpdateFreqHz = 200;
		// motionMagicVoltage.UpdateFreqHz = 200;
    // positionVoltage.UpdateFreqHz = 200;

		getConfigurator().apply(cfg);
		// getPosition().setUpdateFrequency(200);
		// getVelocity().setUpdateFrequency(200);
		// getAcceleration().setUpdateFrequency(200);
		// getMotorVoltage().setUpdateFrequency(200);
  }

  private void setSignals() {
    controlModeSignal = getControlMode();
    closedLoopSPSignal = getClosedLoopReference();
    closedLoopErrorSignal = getClosedLoopError();
    positionSignal = getPosition();
    velocitySignal = getVelocity();
    accelerationSignal = getAcceleration();
    voltageSignal = getMotorVoltage();
    
    lastControlMode = controlModeSignal.getValue().toString();
    lastClosedLoopSP = closedLoopSPSignal.getValueAsDouble();
    lastClosedLoopError = closedLoopErrorSignal.getValueAsDouble();
    lastPosition = positionSignal.getValueAsDouble();
    lastVelocity = velocitySignal.getValueAsDouble();
    lastAcceleration = accelerationSignal.getValueAsDouble();
  }

  private void addLog() {    
    LogManager.addEntry(name + "/Position", getPosition(), 2);
    LogManager.addEntry(name + "/Velocity", getVelocity(), 2);
    LogManager.addEntry(name + "/Acceleration", getAcceleration(), 2);
    LogManager.addEntry(name + "/Voltage", getMotorVoltage(), 2);
    LogManager.addEntry(name + "/Current", getStatorCurrent(), 2);
    LogManager.addEntry(name + "/CloseLoopError", getClosedLoopError(), 2);
    // LogManager.addEntry(name + "/CloseLoopOutput", getClosedLoopOutput(), 1);
    // LogManager.addEntry(name + "/CloseLoopP", getClosedLoopProportionalOutput(), 1);
    // LogManager.addEntry(name + "/CloseLoopI", getClosedLoopIntegratedOutput(), 1);
    // LogManager.addEntry(name + "/CloseLoopD", getClosedLoopDerivativeOutput(), 1);
    // LogManager.addEntry(name + "/CloseLoopFF", getClosedLoopFeedForward(), 1);
    LogManager.addEntry(name + "/CloseLoopSP", getClosedLoopReference(), 2);
  }

  public void checkElectronics() {
    if (getFaultField().getValue() != 0) {
      LogManager.log(name + " have fualt num: " + getFaultField().getValue(), AlertType.kError);
    }
  }

  /**
   * change the slot of the pid and feed forward.
   * will not work if the slot is null
   * @param slot the wanted slot between 0 and 2
   */
  public void changeSlot(int slot) {    
    if (slot < 0 || slot > 2) {
      LogManager.log("slot is not between 0 and 2", AlertType.kError); 
      return;
    }
    if (slot == 0 && config.pid == null) {
      LogManager.log("slot is null, add config for slot 0", AlertType.kError);
      return;
    }
    if (slot == 1 && config.pid1 == null) {
      LogManager.log("slot is null, add config for slot 1", AlertType.kError);
      return;
    }
    if (slot == 2 && config.pid2 == null) {
      LogManager.log("slot is null, add config for slot 2", AlertType.kError);
      return;
    }
    velocityVoltage.withSlot(slot);
    motionMagicVoltage.withSlot(slot);
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
   * @param velocity the wanted velocity in meter per second or radians per seconds depending on the config
   * @param feedForward wanted feed forward to add to the ks kv ka and kg, defaults to 0
   */
  public void setVelocity(double velocity, double feedForward) {
    setControl(velocityVoltage.withVelocity(velocity).withFeedForward(feedForward));
    // velocityEntry.log(velocity);
  }

	public void setVelocity(double velocity) {
		setVelocity(velocity, 0);
	}

 /**
  * set motion magic with PID and Ff
  * <br></br>
  * must add to config motion magic configs (vel, acc, jerk[optional])
  * @param position the wanted position in meter or radians depending on the config
  * @param feedForward wanted feed forward to add to the ks kv ka and kg defaults to 0
  */
	public void setMotionMagic(double position, double feedForward) {
		setControl(motionMagicVoltage.withPosition(position).withFeedForward(feedForward));
		// positionEntry.log(position);
	}

	public void setMotionMagic(double position) {
		setMotionMagic(position, 0);
	}

  public void setPositionVoltage(double position, double feedForward) {
    setControl(positionVoltage.withPosition(position).withFeedForward(feedForward));
    // positionEntry.log(position);
  }

  public void setPositionVoltage(double position) {
    setPositionVoltage(position, 0);
  }

	public void setVelocityWithFeedForward(double velocity) {
    setVelocity(velocity, velocityFeedForward(velocity));
  }

	public void setMotionMagicWithFeedForward(double velocity) {
		setVelocity(velocity, positionFeedForward(velocity));
	}

  private double velocityFeedForward(double velocity) {
    return velocity * velocity * Math.signum(velocity) * config.kv2;
  }
  
  private double positionFeedForward(double positin) {
    return Math.sin(positin*config.posToRad)*config.kSin;
  }

  @SuppressWarnings("rawtypes")
  private double getStatusSignal(StatusSignal statusSignal, double lastValue) {
    statusSignal.refresh();
    if (statusSignal.getStatus() == StatusCode.OK) {
      lastValue = statusSignal.getValueAsDouble();
    }
    return lastValue;
  }

  public String getCurrentControlMode() {
    controlModeSignal.refresh();
    if (controlModeSignal.getStatus() == StatusCode.OK) {
      lastControlMode = controlModeSignal.getValue().toString();
    }
    return lastControlMode;
  }

  public double getCurrentClosedLoopSP() {
    return getStatusSignal(closedLoopSPSignal, lastClosedLoopSP);
  }

  public double getCurrentClosedLoopError() {
    return getStatusSignal(closedLoopErrorSignal, lastClosedLoopError);
  }

	public double getCurrentPosition() {
    return getStatusSignal(positionSignal, lastPosition);
	}

	public double getCurrentVelocity() {
    return getStatusSignal(velocitySignal, lastVelocity);
	}
  
  public double getCurrentAcceleration() {
    return getStatusSignal(accelerationSignal, lastAcceleration);
  }

  public double getCurrentVoltage() {
    return getStatusSignal(voltageSignal, lastVoltage);
  }
	  
  /**
   * creates a widget in elastic of the pid and ff for hot reload
   * @param slot the slot of the close loop perams (from 0 to 2)
   */
  public void configPidFf(int slot) {

    Command configPidFf = new InstantCommand(()-> {
      SlotConfigs cfg = new SlotConfigs();
      cfg.SlotNumber = slot;
      switch (slot) {
        case 0:
          cfg.kP = config.pid.kp;
          cfg.kI = config.pid.ki;
          cfg.kD = config.pid.kd;
          cfg.kS = config.pid.ks;
          cfg.kV = config.pid.kv;
          cfg.kA = config.pid.ka;
          cfg.kG = config.pid.kg;
          break;

        case 1:
          cfg.kP = config.pid1.kp;
          cfg.kI = config.pid1.ki;
          cfg.kD = config.pid1.kd;
          cfg.kS = config.pid1.ks;
          cfg.kV = config.pid1.kv;
          cfg.kA = config.pid1.ka;
          cfg.kG = config.pid1.kg;
          break;

        case 2:
          cfg.kP = config.pid2.kp;
          cfg.kI = config.pid2.ki;
          cfg.kD = config.pid2.kd;
          cfg.kS = config.pid2.ks;
          cfg.kV = config.pid2.kv;
          cfg.kA = config.pid2.ka;
          cfg.kG = config.pid2.kg;
          break;
      
        default:
          cfg.kP = config.pid.kp;
          cfg.kI = config.pid.ki;
          cfg.kD = config.pid.kd;
          cfg.kS = config.pid.ks;
          cfg.kV = config.pid.kv;
          cfg.kA = config.pid.ka;
          cfg.kG = config.pid.kg;
          break;
      }

      getConfigurator().apply(cfg);
    }).ignoringDisable(true);

    SmartDashboard.putData(name + "/PID+FF config", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PID+FF Config");
        
        switch(slot) {
          case 0:
            builder.addDoubleProperty("KP", ()-> config.pid.kp, (double newValue) -> config.pid.kp = newValue);
            builder.addDoubleProperty("KI", ()-> config.pid.ki, (double newValue) -> config.pid.ki = newValue);
            builder.addDoubleProperty("KD", ()-> config.pid.kd, (double newValue) -> config.pid.kd = newValue);
            builder.addDoubleProperty("KS", ()-> config.pid.ks, (double newValue) -> config.pid.ks = newValue);
            builder.addDoubleProperty("KV", ()-> config.pid.kv, (double newValue) -> config.pid.kv = newValue);
            builder.addDoubleProperty("KA", ()-> config.pid.ka, (double newValue) -> config.pid.ka = newValue);
            builder.addDoubleProperty("KG", ()-> config.pid.kg, (double newValue) -> config.pid.kg = newValue);
            break;

          case 1:
            builder.addDoubleProperty("KP", ()-> config.pid1.kp, (double newValue) -> config.pid1.kp = newValue);
            builder.addDoubleProperty("KI", ()-> config.pid1.ki, (double newValue) -> config.pid1.ki = newValue);
            builder.addDoubleProperty("KD", ()-> config.pid1.kd, (double newValue) -> config.pid1.kd = newValue);
            builder.addDoubleProperty("KS", ()-> config.pid1.ks, (double newValue) -> config.pid1.ks = newValue);
            builder.addDoubleProperty("KV", ()-> config.pid1.kv, (double newValue) -> config.pid1.kv = newValue);
            builder.addDoubleProperty("KA", ()-> config.pid1.ka, (double newValue) -> config.pid1.ka = newValue);
            builder.addDoubleProperty("KG", ()-> config.pid1.kg, (double newValue) -> config.pid1.kg = newValue);
            break;

          case 2:
            builder.addDoubleProperty("KP", ()-> config.pid2.kp, (double newValue) -> config.pid2.kp = newValue);
            builder.addDoubleProperty("KI", ()-> config.pid2.ki, (double newValue) -> config.pid2.ki = newValue);
            builder.addDoubleProperty("KD", ()-> config.pid2.kd, (double newValue) -> config.pid2.kd = newValue);
            builder.addDoubleProperty("KS", ()-> config.pid2.ks, (double newValue) -> config.pid2.ks = newValue);
            builder.addDoubleProperty("KV", ()-> config.pid2.kv, (double newValue) -> config.pid2.kv = newValue);
            builder.addDoubleProperty("KA", ()-> config.pid2.ka, (double newValue) -> config.pid2.ka = newValue);
            builder.addDoubleProperty("KG", ()-> config.pid2.kg, (double newValue) -> config.pid2.kg = newValue);
            break;

          default:
            builder.addDoubleProperty("KP", ()-> config.pid.kp, (double newValue) -> config.pid.kp = newValue);
            builder.addDoubleProperty("KI", ()-> config.pid.ki, (double newValue) -> config.pid.ki = newValue);
            builder.addDoubleProperty("KD", ()-> config.pid.kd, (double newValue) -> config.pid.kd = newValue);
            builder.addDoubleProperty("KS", ()-> config.pid.ks, (double newValue) -> config.pid.ks = newValue);
            builder.addDoubleProperty("KV", ()-> config.pid.kv, (double newValue) -> config.pid.kv = newValue);
            builder.addDoubleProperty("KA", ()-> config.pid.ka, (double newValue) -> config.pid.ka = newValue);
            builder.addDoubleProperty("KG", ()-> config.pid.kg, (double newValue) -> config.pid.kg = newValue);
        }
        
        builder.addBooleanProperty("Update", ()-> configPidFf.isScheduled(), 
          value -> {
            if (value) {
              if (!configPidFf.isScheduled()) {
                configPidFf.schedule();
              }
            } else {
              if (configPidFf.isScheduled()) {
                configPidFf.cancel();
              }
            }
          }
        );
      }
    });
  }

  /**
   * creates a widget in elastic to configure motion magic in hot reload
   */
  public void configMotionMagic() {
    Command configMotionMagic = new InstantCommand(()-> {
      MotionMagicConfigs cfg = new MotionMagicConfigs();
      
      cfg.MotionMagicCruiseVelocity = config.motionMagicVelocity;
      cfg.MotionMagicAcceleration = config.motionMagicAccel;
      cfg.MotionMagicJerk = config.motionMagicJerk;
      
      getConfigurator().apply(cfg);
    }).ignoringDisable(true);
    
    SmartDashboard.putData(name + "/Motion Magic Config", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motion Magic Config");
        
        builder.addDoubleProperty("Vel", ()-> config.motionMagicVelocity, value-> config.motionMagicVelocity = value);
        builder.addDoubleProperty("Acc", ()-> config.motionMagicAccel, value-> config.motionMagicAccel = value);
        builder.addDoubleProperty("Jerk", ()-> config.motionMagicJerk, value-> config.motionMagicJerk = value);
        
        builder.addBooleanProperty("Update", ()-> configMotionMagic.isScheduled(), 
        value -> {
          if (value) {
            if (!configMotionMagic.isScheduled()) {
              configMotionMagic.schedule();
            }
          } else {
            if (configMotionMagic.isScheduled()) {
              configMotionMagic.cancel();
            }
          }
        }
        );
      }
    });
  }
  
  /**
   * override the sendable of the talonFX to our costum widget in elastic
   * <br></br>
   * to activate put in the code: <pre> SmartDashboard.putData("talonMotor name", talonMotor);</pre>
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("TalonMotor");
    // builder.addStringProperty("ControlMode", this::getCurrentControlMode, null);
    builder.addBooleanProperty("IsInverted", ()-> config.inverted, null);
    builder.addDoubleProperty("CloseLoopSP", this::getCurrentClosedLoopSP, null);
    builder.addDoubleProperty("CloseLoopError", this::getCurrentClosedLoopError,null);
    builder.addDoubleProperty("Position", this::getCurrentPosition, null);
    builder.addDoubleProperty("Velocity", this::getCurrentVelocity,null);
    builder.addDoubleProperty("Acceleration", this::getCurrentAcceleration,null);
    builder.addDoubleProperty("Voltage", this::getCurrentVoltage, null);
  }
}
