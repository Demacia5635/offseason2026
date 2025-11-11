package frc.demacia.utils.Sensors;

import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.demacia.utils.UpdateArray;
import frc.demacia.utils.Log.LogManager;

public class DigitalEncoder extends DutyCycleEncoder implements SensorInterface{
    DigitalEncoderConfig config;
    String name;

    public DigitalEncoder(DigitalEncoderConfig config){
        super(config.channel, config.scalar, config.offset);
        this.config = config;
        name = config.name;
        configEncoder();
        addLog();
        LogManager.log(name + " digital encoder initialized");
    }
    
    private void configEncoder() {
        setDutyCycleRange(config.minRange, config.maxRange);
        setInverted(config.isInverted);
        setAssumedFrequency(config.frequency);
    }

    private void addLog() {
        LogManager.addEntry(name + "/Position", this::get, 3);
    }

    public String getName(){
        return config.name;
    }

    public void checkElectronics() {
        if (!isConnected()) {
            LogManager.log(name + " encoder disconnected", AlertType.kWarning);
        }
    }
    
    @Override
    public double get(){
        return super.get() * 2 * Math.PI;
    }

    public boolean isConnected() {
        return super.isConnected();
    }

    public void showConfigMotorCommand() {
        UpdateArray.show(name + " CONFIG",
            new String[] {
                "pitch Offset",
                "roll Offset",
                "yaw Offset",
                "x Scalar",
                "y Scalar",
                "z Scalar",
                "is Inverted (1, 0)"
            }, 
            new double[] {
                config.frequency,
                config.minRange,
                config.maxRange,
                config.isInverted ? 1.0 : 0.0,
                config.offset,
                config.scalar
            },
            (double[] array) -> {
                config.withFrequency(array[0])
                .withMinRange(array[1])
                .withMaxRange(array[2])
                .withInvert(array[3] > 0.5)
                .withOffset(array[4])
                .withScalar(array[5]);
                
                configEncoder();
            }
        );
    }
}