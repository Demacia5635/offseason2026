package frc.Demacia.utils.Motors;

import java.util.Random;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RandomPowerGenerator {

    enum Strategy {Stright, Step, Random};

    double maxPower;
    double minPower;
    double maxRiseTime;

    Strategy strategy;
    double maxChange;
    double lastPower;
    double direction;
    double nextTarget;
    double stepEndTime;
    double lastTime;
    Random random;

    public RandomPowerGenerator(double min, double max, double ramp) {
        maxPower = max;
        minPower = min;
        maxRiseTime = ramp;
        random = new Random(System.currentTimeMillis());
        maxChange = 0.02/ramp*12;
        strategy = nextStrategy();
        lastPower = 0;
        direction = 1;
        nextTarget = maxPower;
        lastTime = time();
        stepEndTime = lastTime;
    }

    public void reset() {
        lastPower = 0;
        lastTime = time();
        stepEndTime = lastTime;
        direction = 1;
    }

    private Strategy nextStrategy() {
        double r = random.nextDouble();
        return r > 0.66 ? Strategy.Random : r > 0.33 ? Strategy.Step : Strategy.Stright;
    }

    double time() {
        return Timer.getFPGATimestamp();
    }

    private void nextTarget() {
        if(direction == 1.0) {
            direction = -1;
            nextTarget = minPower;
        } else {
            direction = 1.0;
            nextTarget = maxPower;
        }

        strategy = nextStrategy();
        stepEndTime = time();
    }

    private double nextChange() {
        return (random.nextDouble()*1.5 - 0.5) * maxChange * direction;
    }

    public double next() {
        if((lastPower >= nextTarget && direction > 0) || 
            (lastPower <= minPower && direction < 0 )) {
            nextTarget();
            return lastPower;
        }

        double time = time();
        switch (strategy) {
            case Random:
                lastPower = lastPower + nextChange();
                break;
            case Step:
                if(time > stepEndTime) {
                    stepEndTime = time + random.nextDouble()*0.5;
                    lastPower += random.nextDouble()*maxChange*direction;
                }
                break;
            case Stright:
                lastPower += random.nextDouble()*maxChange*direction;
                break;
            default:
                break;
        }
        lastPower = MathUtil.clamp(lastPower, minPower, maxPower);
        lastTime = time;
        return lastPower;
    }

    public static Command getRandomPowerCommand(MotorInterface motor, SlowPowerGenerator generator, Subsystem subsystem) {
        return new RunCommand(()->motor.setVoltage(generator.next()), subsystem);
    }
 
}
