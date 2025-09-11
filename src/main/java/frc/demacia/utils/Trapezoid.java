package frc.Demacia.utils;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

public class Trapezoid {
    double maxVelocity; // Maximum permissible velocity
    double maxAcceleration; // Maximum permissible acceleration
    private double deltaVelocity; // Velocity increment at each time step
    private double lastTime  = 0;
    private double lastV;
    private double lastA;
    public boolean debug = false;
    double CYCLE_DT = RobotContainer.CYCLE_TIME;
    double CYCLE_DT_SQR = CYCLE_DT * CYCLE_DT;


    // Constructor to initialize with maximum velocity and acceleration
    public Trapezoid(double maxVelocity, double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        deltaVelocity = maxAcceleration * CYCLE_DT;
    }

    // Helper function to calculate distance required to change from current velocity to target velocity
    private double distanceToVelocity(double currentVelocity, double targetVelocity, double acceleration) {
        double t = (currentVelocity-targetVelocity) / maxAcceleration;
        return Math.abs(currentVelocity*t) + 0.5*maxAcceleration*t*t;
    }

    // Function to calculate the next velocity setpoint, based on remaining distance and current and target velocities
    public double calculate(double remainingDistance, double curentVelocity, double targetVelocity) {
        // Case for negative remaining distance
        if(remainingDistance < 0) {
            return  -calculate(-remainingDistance, -curentVelocity, -targetVelocity);
        }

        double time = Timer.getFPGATimestamp();
        // set current velocity to lastV if in same cycle
        if(time - lastTime <= CYCLE_DT && 
                (lastA > 0 && curentVelocity < lastV) || 
                (lastA < 0 && curentVelocity > lastV)) {
            curentVelocity = lastV;
        }        
        
        double v = Math.min(curentVelocity + deltaVelocity, maxVelocity);
        if(curentVelocity < maxVelocity &&
            distanceToVelocity(v, targetVelocity, maxAcceleration) + cycleDistance(curentVelocity, maxAcceleration) < remainingDistance ) {
                // can accelrate
                lastV = v;
        } else if(distanceToVelocity(curentVelocity, targetVelocity, maxAcceleration) + cycleDistance(curentVelocity, 0) < remainingDistance ) {
            // can keep velocity
            lastV = Math.min(curentVelocity, maxVelocity);
        } else {
            // need to deaccelrate
            double t = remainingDistance * 2 / (curentVelocity + targetVelocity);
            double a = (curentVelocity - targetVelocity)/t;
            lastV = Math.min(maxVelocity,curentVelocity - a*CYCLE_DT);
        }
        lastA = lastV - curentVelocity;
        lastTime = time;
        return lastV;
    }

    // Helper function to compute the distance travelled in one cycle
    private double cycleDistance(double velocity, double acceleration ) {
        return velocity * CYCLE_DT + 0.5 * acceleration * CYCLE_DT_SQR;
    }
}