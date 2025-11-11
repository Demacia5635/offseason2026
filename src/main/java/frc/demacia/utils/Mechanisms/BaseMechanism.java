package frc.demacia.utils.Mechanisms;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Motors.MotorInterface;
import frc.demacia.utils.Sensors.SensorInterface;

public abstract class BaseMechanism<T extends BaseMechanism<T>> extends SubsystemBase{

    public static class Trigger {
        private final Supplier<Boolean> condition;
        private final BiConsumer<Pair<MotorInterface[], SensorInterface[]>, double[]> consumer;

        public Trigger(Supplier<Boolean> condition, BiConsumer<Pair<MotorInterface[], SensorInterface[]>, double[]> consumer) {
            this.condition = condition;
            this.consumer = consumer;
        }

        public boolean check() {
            return condition.get();
        }

        public BiConsumer<Pair<MotorInterface[], SensorInterface[]>, double[]> getConsumer() {
            return consumer;
        }
    }

    protected String name;
    protected MotorInterface[] motors;
    protected SensorInterface[] sensors;
    protected double[] values;

    
    private List<Trigger> triggers = new ArrayList<>();
    private BiConsumer<Pair<MotorInterface[], SensorInterface[]>, double[]> consumer;

    public BaseMechanism(String name, MotorInterface[] motors, SensorInterface[] sensors, BiConsumer <Pair<MotorInterface[], SensorInterface[]>, double[]> consumer) {
        this.name = name;
        this.motors = motors != null ? motors : new MotorInterface[0];;
        this.sensors = sensors != null ? sensors : new SensorInterface[0];
        this.consumer = consumer;
        values = new double[0];
    }

    public String getName(){
        return name;
    }

    public void setValues(double[] values){
        this.values = values != null ? values : new double[0];
    }



    @SuppressWarnings("unchecked")
    public T addTrigger(Supplier<Boolean> condition, BiConsumer<Pair<MotorInterface[], SensorInterface[]>, double[]> consumer) {
        if (condition == null || consumer == null) {
            throw new IllegalArgumentException("Trigger condition and consumer cannot be null");
        }
        triggers.add(new Trigger(condition, consumer));
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T addStop(Supplier<Boolean> condition) {
        if (condition == null) {
            throw new IllegalArgumentException("Stop condition cannot be null");
        }
        BiConsumer<Pair<MotorInterface[], SensorInterface[]>, double[]> stopConsumer = 
        (electronics, values) -> {
            for (MotorInterface motor : electronics.getFirst()) {
                motor.setDuty(0);
            }};
        triggers.add(new Trigger(condition, stopConsumer));
        return (T) this;
    }
    
    public Command runMechanismCommand(){
        return new RunCommand(() -> {
            checkTriggers();
            runMechanism();}
            , this);
    }

    private void runMechanism(){
        if (consumer == null) {
            System.err.println("Consumer is null, cannot run mechanism");
            return;
        }
        if (values == null) {
            System.err.println("Values is null, cannot run mechanism");
            return;
        }

        consumer.accept(new Pair<>(motors, sensors), values);
    }

    private void checkTriggers() {
        for (Trigger trigger : triggers) {
            if (trigger.check()) {
                trigger.getConsumer().accept(new Pair<>(motors, sensors), values);
            }
        }
    }

    public void stopAll(){
        if (motors == null) return;
        for (MotorInterface motor : motors){
            motor.setDuty(0);
        }
    }

    public void stop(int motorIndex){
        if (isValidMotorIndex(motorIndex)){
            motors[motorIndex].setDuty(0);
        }
    }

    public void setPower(int motorIndex, double power){
        if (isValidMotorIndex(motorIndex)){
            motors[motorIndex].setDuty(power);
        }
    }

    public void setNeutralModeAll(boolean isBrake) {
        if (motors == null) return;
        for (MotorInterface motor : motors) {
            if (motor != null) motor.setNeutralMode(isBrake);
        }
    }

    public void setNeutralMode(int motorIndex, boolean isBrake){
        if (isValidMotorIndex(motorIndex)){
            motors[motorIndex].setNeutralMode(isBrake);
        }
    }

    public void checkElectronicsAll() {
        if (motors == null) return;
        for (MotorInterface motor : motors) {
            if (motor != null) motor.checkElectronics();
        }
        if (sensors == null) return;
        for (SensorInterface sensor : sensors) {
            if (sensor != null) sensor.checkElectronics();
        }
    }

    public void checkElectronicsMotor(int motorIndex){
        if (isValidMotorIndex(motorIndex)){
            motors[motorIndex].checkElectronics();
        }
    }

    public void checkElectronicsSensor(int sensorIndex){
        if (isValidSensorIndex(sensorIndex)){
            sensors[sensorIndex].checkElectronics();
        }
    }

    public MotorInterface getMotor(int index) {
        if (isValidMotorIndex(index)) return motors[index];
        throw new IllegalArgumentException("Invalid motor index " + index);
    }

    public SensorInterface getSensor(int index) {
        if (isValidSensorIndex(index)) return sensors[index];
        throw new IllegalArgumentException("Invalid sensor index " + index);
    }

    private boolean isValidMotorIndex(int index) {
        return index >= 0 && index < motors.length;
    }

    private boolean isValidSensorIndex(int index) {
        return index >= 0 && index < sensors.length;
    }
}
