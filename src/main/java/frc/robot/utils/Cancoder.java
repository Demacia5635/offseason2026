package frc.robot.utils;
 
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class Cancoder extends CANcoder {

    CancoderConfig config;
    String name;

    StatusSignal<Angle> positionSignal;
    StatusSignal<Angle> absPositionSignal;
    StatusSignal<AngularVelocity> velocitySignal;
    
    double lastPosition;
    double lastAbsPosition;
    double lastVelocity;

    public Cancoder(CancoderConfig config) {
        super(config.id, config.canbus);
        this.config = config;
		name = config.name;
		configCancoder();
        setStatusSignals();
        addLog();
		LogManager.log(name + " cancoder initialized");
    }
    
    private void configCancoder() {
        CANcoderConfiguration canConfig = new CANcoderConfiguration();
		canConfig.MagnetSensor.MagnetOffset = config.offset;
        canConfig.MagnetSensor.SensorDirection = config.inverted ? SensorDirectionValue.Clockwise_Positive: SensorDirectionValue.CounterClockwise_Positive;
        getConfigurator().apply(canConfig);
    }
    
    private void setStatusSignals() {
        positionSignal = getPosition();
        absPositionSignal = getAbsolutePosition();
        velocitySignal = getVelocity();

        lastPosition = positionSignal.getValueAsDouble();
        lastAbsPosition = absPositionSignal.getValueAsDouble();
        lastVelocity = velocitySignal.getValueAsDouble();
    }

    public void checkElectronics() {
        if (getFaultField().getValue() != 0) {
            LogManager.log(name + " have a fault: " + getFaultField().getValue());
        }
    }

    private void addLog() {
        // LogManager.addEntry(name + "/Position", positionSignal, 2);
        LogManager.addEntry(name + "/Absolute position", this::getCurrentAbsPosition, 2);
        // LogManager.addEntry(name + "/Velocity", velocitySignal, 2);
        // LogManager.addEntry(name + "/Acceleration", this::getCurrentAcceleration, 2);
    }

    @SuppressWarnings("rawtypes")
    private double getStatusSignal(StatusSignal statusSignal, double lastValue) {
        statusSignal.refresh();
        if (statusSignal.getStatus() == StatusCode.OK) {
            lastValue = statusSignal.getValueAsDouble() * 2 * Math.PI;
        }
        return lastValue;
    }
    
    /**
     * when the cancoder opens its start at the absolute position
     * @return the none absolute amaunt of rotations the motor did in Radians
     */
    public double getCurrentPosition() {
        return getStatusSignal(positionSignal, lastPosition);
    }
    /**
     * @return the absolute amaunt of rotations the motor did in Radians
     */
    public double getCurrentAbsPosition() {
        return getStatusSignal(absPositionSignal, lastAbsPosition);
    }
    /** 
     * @return the amount of rotations the motor do per second in Radians
     */
    public double getCurrentVelocity(){
        return getStatusSignal(velocitySignal, lastVelocity);
    }

    public double getCurrentAcceleration() {
        velocitySignal.refresh();
        if (velocitySignal.getStatus() == StatusCode.OK) {
            return (velocitySignal.getValueAsDouble() * 2 * Math.PI) - lastVelocity;
        }
        return 0;
    }
}
