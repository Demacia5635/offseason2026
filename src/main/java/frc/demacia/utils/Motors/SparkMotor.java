package frc.demacia.utils.Motors;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.UpdateArray;
import frc.demacia.utils.Utilities;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;
import frc.robot.RobotContainer;

public class SparkMotor extends SparkMax implements Sendable, MotorInterface {

  private SparkConfig config;
  private String name;
  private SparkMaxConfig cfg;
  private int slot = 0;
  private ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;
  private ControlType controlType = ControlType.kDutyCycle;

  private String lastControlMode = "";
  private double lastVelocity;
  private double lastAcceleration;
  private double setPoint = 0;
  private int lastCycleNum = 0;
  private double lastTime = 0;

  public SparkMotor(SparkConfig config) {
    super(config.id, SparkLowLevel.MotorType.kBrushless);
    this.config = config;
    name = config.name;
    configMotor();
    addLog();
    LogManager.log(name + " motor initialized");
    SmartDashboard.putData(name, this);
  }

  private void configMotor() {
    cfg = new SparkMaxConfig();
    cfg.smartCurrentLimit((int) config.maxCurrent);
    cfg.openLoopRampRate(config.rampUpTime);
    cfg.closedLoopRampRate(config.rampUpTime);
    cfg.inverted(config.inverted);
    cfg.idleMode(config.brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    cfg.voltageCompensation(config.maxVolt);
    cfg.encoder.positionConversionFactor(config.motorRatio);
    cfg.encoder.velocityConversionFactor(config.motorRatio / 60);
    updatePID(false);
    if (config.maxVelocity != 0) {
      cfg.closedLoop.maxMotion.maxVelocity(config.maxVelocity).maxAcceleration(config.maxAcceleration);
    }
    this.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void updatePID(boolean apply) {
    cfg.closedLoop.pidf(config.pid[0].kp(), config.pid[0].ki(), config.pid[0].kd(), config.pid[0].kv(),
        ClosedLoopSlot.kSlot0);
    cfg.closedLoop.pidf(config.pid[1].kp(), config.pid[1].ki(), config.pid[1].kd(), config.pid[1].kv(),
        ClosedLoopSlot.kSlot1);
    cfg.closedLoop.pidf(config.pid[2].kp(), config.pid[2].ki(), config.pid[2].kd(), config.pid[2].kv(),
        ClosedLoopSlot.kSlot2);
    if (apply) {
      this.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  @SuppressWarnings("unchecked")
  private void addLog() {
    LogManager.addEntry(name + "/Position and Velocity and Acceleration and Voltage and Current and CloseLoopError and CloseLoopSP", 
      () -> new double[] {
        getCurrentPosition(),
        getCurrentVelocity(),
        getCurrentAcceleration(),
        getCurrentVoltage(),
        getCurrentCurrent(),
        getCurrentClosedLoopError(),
        getCurrentClosedLoopSP(),
      }).withLogLevel(LogLevel.LOG_ONLY_NOT_IN_COMP).build();
  }

  public void checkElectronics() {
      Faults faults = getFaults();
      if (faults != null) {
          LogManager.log(name + " have fault num: " + faults.toString(), AlertType.kError);
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
    this.closedLoopSlot = slot == 0 ? ClosedLoopSlot.kSlot0 : slot == 1 ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot2;
  }

  /*
   * set motor to brake or coast
   */
  public void setNeutralMode(boolean isBrake) {
    cfg.idleMode(isBrake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * set power from 1 to -1 (v/12) no PID/FF
   * 
   * @param power the wanted power between -1 to 1
   */
  public void setDuty(double power) {
    super.set(power);
    controlType = ControlType.kDutyCycle;
    lastControlMode = "Duty Cycle";
  }

  public void setVoltage(double voltage) {
    super.setVoltage(voltage);
    controlType = ControlType.kVoltage;
    lastControlMode = "Voltage";
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
    if (config.maxVelocity == 0) {
      LogManager.log(name + ": maxVelocity not configured", AlertType.kError);
      return;
    }
    getClosedLoopController().setReference(velocity, ControlType.kMAXMotionVelocityControl, closedLoopSlot, feedForward);
    controlType = ControlType.kMAXMotionVelocityControl;
    lastControlMode = "Velocity";
    setPoint = velocity;
  }

  public void setVelocity(double velocity) {
    setVelocity(velocity, config.pid[closedLoopSlot.value].ks()*Math.signum(velocity));
  }

  public void setPositionVoltage(double position, double feedForward) {
    getClosedLoopController().setReference(position, ControlType.kPosition, closedLoopSlot, feedForward);
    controlType = ControlType.kPosition;
    lastControlMode = "Position Voltage";
    setPoint = position;
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

  @Override
  public void setMotion(double position, double feedForward) {
    if (config.maxVelocity == 0) {
      LogManager.log(name + ": maxVelocity not configured", AlertType.kError);
      return;
    }
    getClosedLoopController().setReference(position, ControlType.kMAXMotionPositionControl, closedLoopSlot, feedForward);
    controlType = ControlType.kMAXMotionPositionControl;
    lastControlMode = "Motion";
    setPoint = position;
  }

  @Override
  public void setMotion(double position) {
    setMotion(position, config.pid[closedLoopSlot.value].ks()*Utilities.signumWithDeadband(position - getCurrentPosition(), 0.5));
  }

  @Override
  public void setAngle(double angle, double feedForward) {
    setMotion(MotorUtils.getPositionForAngle(getCurrentPosition(), angle, config.isRadiansMotor), feedForward);
    lastControlMode = "Angle";
  }
  @Override
  public void setAngle(double angle) {
    setMotion(MotorUtils.getPositionForAngle(getCurrentPosition(), angle, config.isRadiansMotor));
  }

  private double velocityFeedForward(double velocity) {
    return velocity * velocity * Math.signum(velocity) * config.kv2;
  }

  private double positionFeedForward(double position) {
    return Math.cos(position * config.posToRad) * config.kSin;
  }

  @Override
  public String getCurrentControlMode() {
    return lastControlMode;
  }

  @Override
  public double getCurrentClosedLoopError() {
    switch (controlType) {
      case kPosition, kMAXMotionPositionControl:
        return setPoint - getCurrentPosition();
      case kVelocity, kMAXMotionVelocityControl:
        return setPoint - getCurrentVelocity();
      default:
        return 0;
    }
  }

  @Override
  public double getCurrentClosedLoopSP() {
    return setPoint;
  }

  /**
   * creates a widget in elastic of the pid and ff for hot reload
   * 
   * @param slot the slot of the close loop perams (from 0 to 2)
   */
  public void showConfigPIDFSlotCommand(int slot) {
    CloseLoopParam p = config.pid[slot];
    if (p != null) {
      UpdateArray.show(name + " PID " + slot, CloseLoopParam.PARAMETER_NAMES, p.toArray(), (double[] array) -> updatePID(true));
    }
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
              changeSlot(slot);
  
              configMotor();
  
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

  public double getCurrentPosition() {
    return getEncoder().getPosition();
  }

  public double getCurrentAngle() {
    if (config.isRadiansMotor) {
      return MathUtil.angleModulus(getCurrentPosition());
    } else if (config.isDegreesMotor) {
      return MathUtil.inputModulus(getCurrentPosition(), -180, 180);
    }
    return 0;
  }

  public double getCurrentVelocity() {
    double velocity = getEncoder().getVelocity();
    if (lastCycleNum != RobotContainer.N_CYCLE) {
      lastCycleNum = RobotContainer.N_CYCLE;
      double time = Timer.getFPGATimestamp();
      lastAcceleration = (velocity - lastVelocity) / (time - lastTime);
      lastTime = time;
      lastVelocity = velocity;
    }
    return velocity;
  }

  public double getCurrentAcceleration() {
    return lastAcceleration;
  }

  public double getCurrentVoltage() {
    return getAppliedOutput() * 12;
  }
  public double getCurrentCurrent() {
    return getOutputCurrent();
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
    builder.setSmartDashboardType("Spark Motor");
    builder.addStringProperty("ControlMode", this::getCurrentControlMode, null);
    builder.addDoubleProperty("Position", this::getCurrentPosition, null);
    builder.addDoubleProperty("Velocity", this::getCurrentVelocity, null);
    builder.addDoubleProperty("Voltage", this::getCurrentVoltage, null);
    builder.addDoubleProperty("Current", this::getCurrentCurrent, null);
    builder.addDoubleProperty("CloseLoop Error", this::getCurrentClosedLoopError, null);
    if (config.isDegreesMotor || config.isRadiansMotor) {
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
    getEncoder().setPosition(position);
  }

  @Override
  public void showConfigMotionVelocitiesCommand() {
    UpdateArray.show(name + "MOTION PARAM",
        new String[] { "Velocity", "Acceleration" },
        new double[] { config.maxVelocity, config.maxAcceleration },
        (double[] array) -> {
          config.maxVelocity = array[0];
          config.maxAcceleration = array[1];
          cfg.closedLoop.maxMotion.maxVelocity(config.maxVelocity).maxAcceleration(config.maxAcceleration);
          configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        });
  }
}
