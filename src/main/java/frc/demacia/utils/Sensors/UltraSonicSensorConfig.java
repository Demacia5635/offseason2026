package frc.demacia.utils.Sensors;
public class UltraSonicSensorConfig extends AnalogSensorConfig<UltraSonicSensorConfig> {
    int pingChannel;
    public UltraSonicSensorConfig(int channel, int pingChannel, String name) {
        super(channel, name);
        this.pingChannel = pingChannel;
        sensorType = UltraSonicSensor.class;
    }
}