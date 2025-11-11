package frc.demacia.utils.Sensors;
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
    
    
    public int getValue() {
        return super.getValue();
    }

    public double get(){
        return getValue();
    }

    public double getVoltage() {
        return super.getVoltage();
    }

    public void checkElectronics(){
        
    }

}

