package frc.demacia.utils.Motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.UpdateArray;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;

public class TalonSRXMotor extends TalonSRX implements MotorInterface,Sendable {
    TalonSRXConfig config;
    String name;

    int slot = 0;

    String lastControlMode="";

    public TalonSRXMotor(TalonSRXConfig config) {
        super(config.id);
        this.config = config;
        name = config.name;
        configMotor();
        addLog();
        LogManager.log(name + " motor initialized");
        SmartDashboard.putData(name, this);
    }

    private void configMotor() {
        configFactoryDefault();
        configContinuousCurrentLimit((int) config.maxCurrent);
        configPeakCurrentLimit((int) config.maxCurrent);
        configPeakCurrentDuration(100);
        enableCurrentLimit(true);
        configClosedloopRamp(config.rampUpTime);
        configOpenloopRamp(config.rampUpTime);
        setInverted(config.inverted);
        setNeutralMode(config.brake ? NeutralMode.Brake : NeutralMode.Coast);
        configPeakOutputForward(config.maxVolt / 12.0);
        configPeakOutputReverse(config.minVolt / 12.0);
        // cfg.Feedback.SensorToMechanismRatio = config.motorRatio * unitMultiplier;
        updatePID();
        configVoltageCompSaturation(config.maxVolt);
        enableVoltageCompensation(true);
        configureMotionMagic();
    }

    private void updatePID() {
        System.out.println("update PID");
    
        config_kP(0, config.pid[0].kp());
        config_kI(0, config.pid[0].ki());
        config_kD(0, config.pid[0].kd());
        config_kF(0, config.pid[0].kv()*(1023.0 / 12.0));
        config_kP(1, config.pid[1].kp());
        config_kI(1, config.pid[1].ki());
        config_kD(1, config.pid[1].kd());
        config_kF(1, config.pid[1].kv()*(1023.0 / 12.0));
        config_kP(2, config.pid[2].kp());
        config_kI(2, config.pid[2].ki());
        config_kD(2, config.pid[2].kd());
        config_kF(2, config.pid[2].kv()*(1023.0 / 12.0));
    }

    private void configureMotionMagic() {
        configMotionAcceleration((int) (config.maxAcceleration/ config.motorRatio));
        configMotionCruiseVelocity((int) (config.maxVelocity/ config.motorRatio));
    }

    @SuppressWarnings("unchecked")
    private void addLog() {
      LogManager.addEntry(name + "/Position and Velocity and Acceleration and Voltage and Current and CloseLoopError and CloseLoopSP2", 
        () -> new double[] {
          getCurrentPosition(),
          getCurrentVelocity(),
          getCurrentAcceleration(),
          getCurrentVoltage(),
          getCurrentCurrent(),
          getCurrentClosedLoopError(),
          getCurrentClosedLoopSP(),
        }).withLogLevel(LogLevel.LOG_ONLY_NOT_IN_COMP)
        .WithIsMotor().build();
    }

    public void checkElectronics() {
        com.ctre.phoenix.motorcontrol.Faults faults = new com.ctre.phoenix.motorcontrol.Faults();
        getFaults(faults);
        if (faults.hasAnyFault()) {
            LogManager.log(name + " have fault num: " + faults.toString(), AlertType.kError);
        }
    }

    public void changeSlot(int slot){
        if (slot < 0 || slot > 2) {
            LogManager.log("slot is not between 0 and 2", AlertType.kError);
            return;
        }
        this.slot = slot;
    }

    public void setNeutralMode(boolean isBrake){
        setNeutralMode(isBrake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public void setDuty(double power){
        set(ControlMode.PercentOutput, power);
        lastControlMode = "Duty Cycle";
    }

    public void setVoltage(double voltage){
        set(ControlMode.PercentOutput, voltage/12.0);
        lastControlMode = "Voltage";
    }

    public void setVelocity(double velocity, double feedForward){
        selectProfileSlot(slot, 0);
        set(ControlMode.Velocity, velocity / config.motorRatio, DemandType.ArbitraryFeedForward, feedForward / 12.0);
        lastControlMode = "Velocity";
    }

    public void setVelocity(double velocity){
        setVelocity(velocity, 0);
    }

    public void setMotion(double position, double feedForward){
        selectProfileSlot(slot, 0);
        set(ControlMode.MotionMagic, position / config.motorRatio, DemandType.ArbitraryFeedForward, feedForward / 12.0);
        lastControlMode = "Position";
    }

    public void setMotion(double position){
        setMotion(position, 0);
    }
    
    public void setAngle(double angle, double feedForward) {
      setMotion(MotorUtils.getPositionForAngle(getCurrentPosition(), angle, config.isRadiansMotor), feedForward);
      lastControlMode = "Angle";
    }
    
    public void setAngle(double angle) {
      setMotion(MotorUtils.getPositionForAngle(getCurrentPosition(), angle, config.isRadiansMotor));
    }

    @Override
    public void setPositionVoltage(double position, double feedForward) {
        selectProfileSlot(slot, 0);
        set(ControlMode.Position, position / config.motorRatio, DemandType.ArbitraryFeedForward, feedForward / 12.0);
        lastControlMode = "Position Voltage";
    }

    @Override
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

    public String getCurrentControlMode(){
        return lastControlMode;
    }@Override
    public double getCurrentClosedLoopSP() {
        return getClosedLoopTarget(0) * config.motorRatio;
    }

    @Override
    public double getCurrentClosedLoopError() {
        return getClosedLoopError(0) * config.motorRatio;
    }

    @Override
    public double getCurrentPosition() {
        return getSelectedSensorPosition() * config.motorRatio;
    }

    @Override
    public double getCurrentAngle() {
        if (config.isRadiansMotor) {
            return MathUtil.angleModulus(getCurrentPosition());
        } else if (config.isDegreesMotor) {
            return MathUtil.inputModulus(getCurrentPosition(), -180, 180);
        }
        return 0;
    }

    @Override
    public double getCurrentVelocity() {
        return getSelectedSensorVelocity() * config.motorRatio * 10;
    }

    @Override
    public double getCurrentAcceleration() {
        return 0; // Phoenix 5 SRX doesn't have direct acceleration
    }

    @Override
    public double getCurrentVoltage() {
        return getMotorOutputVoltage();
    }

    @Override
    public double getCurrentCurrent() {
        return getStatorCurrent();
    }

    @Override
    public void setEncoderPosition(double position) {
        setSelectedSensorPosition(position / config.motorRatio);
    }

    /**
     * creates a widget in elastic of the pid and ff for hot reload
     * 
     * @param slot the slot of the close loop perams (from 0 to 2)
     */
    public void showConfigPIDFSlotCommand(int slot) {
        CloseLoopParam p = config.pid[slot];
        if(p != null) {
            UpdateArray.show(name + " PID " + slot , CloseLoopParam.PARAMETER_NAMES, p.toArray(),(double[] array)->updatePID());
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
                "Motor Ratio"
            },
            new double[] {
                config.maxCurrent,
                config.rampUpTime,
                config.maxVolt,
                config.brake ? 1.0 : 0.0,
                config.inverted ? 1.0 : 0.0,
                config.motorRatio
            },
            (double[] array) -> {
                config.withCurrent(array[0])
                      .withRampTime(array[1])
                      .withVolts(array[2])
                      .withBrake(array[3] > 0.5)
                      .withInvert(array[4] > 0.5);
    
                config.motorRatio = array[5];
    
                configMotor();
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
   
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Talon SRX Motor");
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

    @Override
    public String name() {
        return name;
    }

    public double gearRatio() {
        return config.motorRatio;
    }
}
