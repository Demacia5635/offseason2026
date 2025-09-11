package frc.Demacia.utils.Motors;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SlowPowerGenerator {

    double stepPower;
    double minPower;
    double waitTime;

    double lastPower;
    double stepEndTime;

    public SlowPowerGenerator(double min, double step, double wait) {
        stepPower = step;
        minPower = min;
        waitTime = wait;
        lastPower = 0;
        stepEndTime = 0;
    }



    double time() {
        return Timer.getFPGATimestamp();
    }

    public double next() {
        if(stepEndTime == 0) {
            stepEndTime = time() + waitTime;
            lastPower = 0;
        } else if(time() > stepEndTime) {
            lastPower = Math.max(minPower, lastPower + stepPower);
            stepEndTime = time() + waitTime;
        }
        return lastPower;
    }

    public void reset() {
        stepEndTime = 0;
        lastPower = 0;
    }

    public static Command getSlowPowerCommand(MotorInterface motor, SlowPowerGenerator generator, Subsystem subsystem) {
        return new RunCommand(()->motor.setVoltage(generator.next()), subsystem)
            .beforeStarting(()->generator.reset(), subsystem)
            .finallyDo(()->motor.setDuty(0));
    }

}
