package frc.demacia.utils.Mechanisms;

import frc.demacia.utils.Motors.MotorInterface;
import frc.demacia.utils.Sensors.SensorInterface;

public class Arm extends StateBasedMechanism<Arm>{
    public interface ArmState extends MechanismState{
        double[] getValues();
    }

    public Arm(String name, MotorInterface[] motors, Class<? extends Enum<? extends ArmState>> enumClass) {
        super(name, motors, new SensorInterface[0], enumClass, 
        (electronics, values) -> {
            MotorInterface[] motor = electronics.getFirst();
            for (int i = 0; i < motor.length && i < values.length; i++) {
                motor[i].setAngle(values[i]);
            }});
    }
}
