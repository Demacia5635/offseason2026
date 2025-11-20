package frc.demacia.utils.Sensors;

import frc.demacia.utils.UpdateArray;
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;
import frc.demacia.utils.Log.LogManager;

public class AnalogEncoder extends edu.wpi.first.wpilibj.AnalogEncoder implements AnalogSensorInterface{
    AnalogEncoderConfig config;
    String name;

    public AnalogEncoder(AnalogEncoderConfig config){
        super(config.channel, config.fullRange, config.offset);
        this.config = config;
        name = config.name;
        configEncoder();
        addLog();
        LogManager.log(name + " analog encoder initialized");
    }

    private void configEncoder() {
        setInverted(config.isInverted);
        setVoltagePercentageRange(config.minRange, config.maxRange);
    }

    @SuppressWarnings("unchecked")
    private void addLog() {
        LogManager.addEntry(name + "/Position", this::get)
        .withLogLevel(LogLevel.LOG_ONLY_NOT_IN_COMP).build();
    }

    public void checkElectronics(){
        
    }

    public String getName(){
        return config.name;
    }

    public boolean isInverted(){
        return config.isInverted;
    }
    
    @Override
    public double get(){
        return super.get();
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
                .withOffset(array[1])
                .withMaxRange(array[2])
                .withMinRange(array[3])
                .withFullRange(array[4]);
                
                configEncoder();
            }
        );
    }
}