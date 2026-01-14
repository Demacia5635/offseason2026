package frc.demacia.utils.Sensors;


public class ColorSensorConfig extends BaseSensorConfig<ColorSensorConfig>{
    public ColorSensorConfig(int channel, String name){
        super(channel, name);
        sensorType = ColorSensor.class;
    }
}