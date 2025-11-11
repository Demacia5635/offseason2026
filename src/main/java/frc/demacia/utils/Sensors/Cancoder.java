package frc.demacia.utils.Sensors;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.demacia.utils.UpdateArray;
import frc.demacia.utils.Log.LogManager;

public class Cancoder extends CANcoder implements AnalogSensorInterface{

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
        canConfig.MagnetSensor.SensorDirection = config.isInverted ? SensorDirectionValue.Clockwise_Positive: SensorDirectionValue.CounterClockwise_Positive;
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
        LogManager.addEntry(name + "Position and Velocity",  () -> new double[] {
            getCurrentAbsPosition(),
            getCurrentPosition(),
            getCurrentVelocity(),
            getCurrentAcceleration()
        }, 3);
    }

    public String getName(){
        return config.name;
    }

    public double get(){
        return getCurrentPosition();
    }
    
    /**
     * when the cancoder opens its start at the absolute position
     * @return the none absolute amaunt of rotations the motor did in Radians
     */
    public double getCurrentPosition() {
        lastPosition = StatusSignalHelper.getStatusSignalWith2Pi(positionSignal, lastPosition);
        return lastPosition;
    }

    public double getCurrentAbsPosition() {
        lastAbsPosition = StatusSignalHelper.getStatusSignalWith2Pi(absPositionSignal, lastAbsPosition);
        return lastAbsPosition;
    }

    public double getCurrentVelocity(){
        lastVelocity = StatusSignalHelper.getStatusSignalWith2Pi(velocitySignal, lastVelocity);
        return lastVelocity;
    }

    public double getCurrentAcceleration() {
        velocitySignal.refresh();
        if (velocitySignal.getStatus() == StatusCode.OK) {
            return (velocitySignal.getValueAsDouble() * 2 * Math.PI) - lastVelocity;
        }
        return 0;
    }

    public void showConfigMotorCommand() {
        UpdateArray.show(name + " CONFIG",
            new String[] {
                "is Inverted (1, 0)",
                "Offset"
            }, 
            new double[] {
                config.isInverted ? 1.0 : 0.0,
                config.offset
            },
            (double[] array) -> {
                config.withInvert(array[0] > 0.5)
                .withOffset(array[1]);
                
                configCancoder();
            }
        );
    }
}