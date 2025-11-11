package frc.demacia.utils.Sensors;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.demacia.utils.Log.LogManager;
public class UltraSonicSensor extends Ultrasonic implements AnalogSensorInterface {
    public UltraSonicSensor( UltraSonicSensorConfig config) {
        super(config.pingChannel, config.channel);
        this.config = config;
        name = config.name;
        addLog();
		LogManager.log(name + " cancoder initialized");
    }

    String name;
    UltraSonicSensorConfig config;
    public double get() {
        return getRangeMeters();
    }


    public String getName() {
        return config.name;
    }
 
    @Override
    public void ping() {
        super.ping();
    }

    public double getRangeMeters() {
        ping();
        return getRangeMM() / 100.0;
    }
    
    private void addLog() {
        LogManager.addEntry(name + "range", () -> getRangeMeters(), 3);
            getRangeMeters();
    }

    public void checkElectronics(){

    }
}
