package frc.demacia.utils.Sensors;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
    public class OpticalSensor extends AnalogInput implements SensorInterface {
        private final OpticalSensorConfig config;
    
        public OpticalSensor(OpticalSensorConfig config) {
            super(config.channel);
            this.config = config;
        }
    
        public String getName() {
            return config.name;  
        }
    
    
    public double getCurrentValue() {
        return getVoltage();
    }

    public double get(){
        return getCurrentValue();
    }

    public double getVoltage() {
        return getVoltage();
    }

    public void checkElectronics(){
        
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("value" , this::get, null);
    }
}

