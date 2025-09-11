package frc.Demacia.utils.Motors;

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
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.Demacia.utils.Utilities;
import frc.Demacia.utils.Elastic.UpdateArray;
import frc.Demacia.utils.Log.LogManager;
import frc.Demacia.utils.Log.MotorLogEntry;
import frc.robot.RobotContainer;

public class SparkMotor extends SparkMax implements Sendable, MotorInterface {

  private SparkConfig config;
  private String name;
  private SparkMaxConfig cfg;
  private ClosedLoopSlot slot = ClosedLoopSlot.kSlot0;
  private ControlType controlType = ControlType.kDutyCycle;

  private String lastControlMode;
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
    SmartDashboard.putData(name, this);
    LogManager.log(name + " motor initialized");
  }

  private void configMotor() {
    cfg = new SparkMaxConfig();
    cfg.smartCurrentLimit((int) config.maxCurrent);
    cfg.openLoopRampRate(config.rampUpTime);
    cfg.closedLoopRampRate(config.rampUpTime);
    cfg.inverted(config.inverted);
    cfg.idleMode(config.brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    cfg.voltageCompensation(config.maxVolt);
    cfg.encoder.positionConversionFactor(1/config.motorRatio);
    cfg.encoder.velocityConversionFactor(1/config.motorRatio / 60);
    updatePID(false);
    if (config.maxVelocity != 0) {
      cfg.closedLoop.maxMotion.maxVelocity(config.maxVelocity).maxAcceleration(config.maxAcceleration);
    }
    getEncoder();
    getClosedLoopController();
    this.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void updatePID(boolean apply) {
    cfg.closedLoop.pidf(config.pid[0].kp()/12.0, config.pid[0].ki()/12.0, config.pid[0].kd()/12.0, config.pid[0].kv()/12.0,
        ClosedLoopSlot.kSlot0);
    cfg.closedLoop.pidf(config.pid[1].kp()/12.0, config.pid[1].ki()/12.0, config.pid[1].kd()/12.0, config.pid[1].kv()/12.0,
        ClosedLoopSlot.kSlot1);
    cfg.closedLoop.pidf(config.pid[2].kp()/12.0, config.pid[2].ki()/12.0, config.pid[2].kd()/12.0, config.pid[2].kv()/12.0,
        ClosedLoopSlot.kSlot2);
    if (apply) {
      this.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  private void addLog() {
    MotorLogEntry.add(this);

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
    this.slot = slot == 0 ? ClosedLoopSlot.kSlot0 : slot == 1 ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot2;
  }

  /*
   * set motor to brake or coast
   */
  public void setNeutralMode(boolean isBrake) {
    cfg.idleMode(config.brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
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

  }

  public void setVoltage(double voltage) {
    super.setVoltage(voltage);
    controlType = ControlType.kVoltage;
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
    super.closedLoopController.setReference(velocity, ControlType.kVelocity, slot, feedForward);
    controlType = ControlType.kVelocity;
    setPoint = velocity;
  }

  public void setVelocity(double velocity) {
    setVelocity(velocity, config.pid[slot.value].ks()/12.0*Math.signum(velocity));
  }

  public void setPositionVoltage(double position, double feedForward) {
    super.closedLoopController.setReference(position, ControlType.kPosition, slot, feedForward);
    controlType = ControlType.kPosition;
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

  private double velocityFeedForward(double velocity) {
    return velocity * velocity * Math.signum(velocity) * config.kv2;
  }

  private double positionFeedForward(double positin) {
    return Math.cos(positin * config.posToRad) * config.kSin;
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
      UpdateArray.show(name + " PID " + slot, CloseLoopParam.names, p.toArray(), (double[] array) -> updatePID(true));
    }
  }

  public double getCurrentPosition() {
    return encoder.getPosition();
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
    double velocity = encoder.getVelocity();
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
 //   builder.setSmartDashboardType("Spark Motor");
//    builder.addStringProperty("ControlMode", this::getCurrentControlMode, null);
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
  public void setMotion(double position, double feedForward) {
    super.closedLoopController.setReference(position, ControlType.kMAXMotionPositionControl, slot, feedForward);
    controlType = ControlType.kMAXMotionPositionControl;
    setPoint = position;
  }

  @Override
  public void setMotion(double position) {
    setMotion(position, config.pid[slot.value].ks()*Utilities.signumWithDeadband(position - getCurrentPosition(), 0.5));
  }

  @Override
  public void setAngle(double angle, double feedForward) {
    setMotion(MotorUtils.getPositionForAngle(getCurrentPosition(), angle, config.isRadiansMotor), feedForward);
  }
  @Override
  public void setAngle(double angle) {
    setMotion(MotorUtils.getPositionForAngle(getCurrentPosition(), angle, config.isRadiansMotor));
  }

  @Override
  public void setEncoderPosition(double position) {
    encoder.setPosition(position);
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

  @Override
  public void showSysidCommands(Subsystem subsystem) {
    MotorUtils.showSysidCommands(this, config, subsystem);
  }

}
