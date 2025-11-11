package frc.demacia.utils.Mechanisms;

import frc.demacia.utils.Motors.MotorInterface;
import frc.demacia.utils.Sensors.SensorInterface;

public class Elevator extends StateBasedMechanism<Elevator>{
    public interface ElevatorState extends MechanismState{
        double[] getValues();
    }

    public Elevator(String name, MotorInterface[] motors, Class<? extends Enum<? extends ElevatorState>> enumClass) {
        super(name, motors, new SensorInterface[0], enumClass, 
        (electronics, values) -> {
            MotorInterface[] motor = electronics.getFirst();
            for (int i = 0; i < motor.length && i < values.length; i++) {
                motor[i].setMotion(values[i]);
            }});
    }
}
