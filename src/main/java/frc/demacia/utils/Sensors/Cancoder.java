package frc.Demacia.utils.Sensors;
 
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.Demacia.utils.StatusSignalData;
import frc.Demacia.utils.Log.LogEntry;
import frc.Demacia.utils.Log.LogManager;
import frc.Demacia.utils.Log.LogSupplier;
import frc.Demacia.utils.Log.LogManager.LOG_TARGET;

public class Cancoder extends CANcoder {

    CancoderConfig config;
    String name;

    CANcoderConfiguration cfg = new CANcoderConfiguration();

    StatusSignalData<Angle> positionSignal;
    StatusSignalData<Angle> absPositionSignal;
    StatusSignalData<AngularVelocity> velocitySignal;
    

    public Cancoder(CancoderConfig config) {
        super(config.id, config.canbus.canbus);
        this.config = config;
		name = config.name;
		configCancoder();
        setStatusSignals();
        addLog();
		LogManager.log(name + " cancoder initialized");
    }
    
    private void configCancoder() {
		cfg.MagnetSensor.MagnetOffset = config.offset;
        cfg.MagnetSensor.SensorDirection = config.inverted ? SensorDirectionValue.Clockwise_Positive: SensorDirectionValue.CounterClockwise_Positive;
        getConfigurator().apply(cfg);
    }
    
    private void setStatusSignals() {
        positionSignal = new StatusSignalData<>(getPosition(),360);
        absPositionSignal = new StatusSignalData<>(getAbsolutePosition(), 360);
        velocitySignal = new StatusSignalData<>(getVelocity(), 360);
    }

    private void addLog() {
        new LogEntry(
            name, 
            new LogSupplier[] {
                new LogSupplier(positionSignal, "Position",null),
                new LogSupplier(absPositionSignal, "AbsPosition",null),
                new LogSupplier(velocitySignal, "Velocity",null),
                            },
            LOG_TARGET.LOG_AND_NT, 
            "Cancoder", 
            "", 
            ""
        );
    }

    /**
     * when the cancoder opens its start at the absolute position
     * @return the none absolute amaunt of rotations the motor did in Radians
     */
    public double getCurrentPosition() {
        return positionSignal.get();
    }
    /**
     * @return the absolute amaunt of rotations the motor did in Radians
     */
    public double getCurrentAbsPosition() {
        return absPositionSignal.get();
    }
    /** 
     * @return the amount of rotations the motor do per second in Radians
     */
    public double getCurrentVelocity(){
        return velocitySignal.get();
    }
}