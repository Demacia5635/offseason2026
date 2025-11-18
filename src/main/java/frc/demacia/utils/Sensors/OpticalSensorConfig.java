package frc.demacia.utils.Sensors;

public class OpticalSensorConfig extends AnalogSensorConfig<OpticalSensorConfig> {

    public OpticalSensorConfig(String name, int port) {
        super(port, name);
        sensorType = OpticalSensor.class;
    }

}
