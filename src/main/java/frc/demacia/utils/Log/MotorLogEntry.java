package frc.Demacia.utils.Log;

import frc.Demacia.utils.Log.LogManager.LOG_TARGET;
import frc.Demacia.utils.Motors.SparkMotor;
import frc.Demacia.utils.Motors.TalonMotor;

public class MotorLogEntry {

    public static void add(TalonMotor motor) {
        new LogEntry(
            motor.name(), 
            new LogSupplier[] {
                new LogSupplier(motor.getVoltageSignal(), "Volts",null),
                new LogSupplier(motor.getPositionSignal(), "Position",null),
                new LogSupplier(motor.getVelocitySignal(), "Velocity",null),
                new LogSupplier(motor.getAccelerationSignal(), "Acceleration",null),
                new LogSupplier(motor.getCurrentSignal(), "Current",null),
                new LogSupplier(motor.getClosedLoopErrorSignal(), "Error",null),
                new LogSupplier(motor.getClosedLoopSPSignal(), "SetPoint",null)
                            },
            LOG_TARGET.LOG_AND_NT, 
            "Motor", 
            "Talon", 
            "GearRatio:" + motor.gearRatio());

    }
    public static void add(SparkMotor motor) {
        new LogEntry(
            motor.name(), 
            new LogSupplier[] {
                new LogSupplier(motor::getCurrentVoltage, "Volts",null),
                new LogSupplier(motor::getCurrentPosition, "Position",null),
                new LogSupplier(motor::getCurrentVelocity, "Velocity",null),
                new LogSupplier(motor::getCurrentAcceleration, "Acceleration",null),
                new LogSupplier(motor::getOutputCurrent, "Current",null),
                new LogSupplier(motor::getCurrentClosedLoopError, "Error",null),
                new LogSupplier(motor::getCurrentClosedLoopSP, "SetPoint",null)
                            },
            LOG_TARGET.LOG_AND_NT, 
            "Motor", 
            "Spark", 
            "GearRatio:" + motor.gearRatio());
    }
}
