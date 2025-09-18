package frc.Demacia.utils;

import edu.wpi.first.math.MathUtil;

public class Trapezoid2 {
    /**
     * Calculate the trapezoid velocity to reach the target position
     * @param remainingDistance
     * @param curentVelocity
     * @param targetVelocity
     * @param maxVelocity
     * @param maxAcceleration
     * @return next cycle velocity
     */
    public static double calculate(double remainingDistance, double curentVelocity, double targetVelocity, double maxVelocity, double maxAcceleration) {
        if(remainingDistance > 0 && targetVelocity >= 0) {
            // calculate the maximum velocity if continuing acceleration/deacceleratin regardless of maxVelocity
            // time to reach maxV = (maxV - currentVelocity) / maxA
            // distance to maxV = average velocity * time = (maxV + currentVelocity)/2 * (maxV - currentVelocity) / maxAcceleration = 
            //         (maxV^2 - currentVelocity^2)/(2 * maxAcceleration)
            // same for distance to target = (maxV^2 - targetVelocity^2)/(2 * maxAcceleration)
            // remainingDistance = (maxV^2 - targetVelocity^2)/(2 * maxAcceleration) + (maxV^2 - currentVelocity^2)/(2 * maxAcceleration)
            // remainingDistance * 2 * maxAcceleration = 2*maxV^2 - targetVelocity^2 - currentVelocity^2
            // maxV^2 = remainingDistance * maxAcceleration + (targetVelocity^2 + currentVelocity^2)/2
            double maxV = Math.sqrt(remainingDistance*maxAcceleration + (curentVelocity*curentVelocity + targetVelocity * targetVelocity)/2);
            // define the max velocity between the calculated and the provided
            maxV = Math.min(maxV, maxVelocity);
            // calculate the deacceleation time from maxV to targetVelocity
            double deaccelrateTime = (maxV - targetVelocity) / maxAcceleration;
            // calculate the required distance for deacceleration
            double startDeaccelraionDistance = (targetVelocity + maxV) * deaccelrateTime / 2;
            // check remaining distance and deacceleration distance
            if(remainingDistance > startDeaccelraionDistance) {
                return Math.min(maxVelocity, curentVelocity + maxAcceleration * 0.02);
            } else { // deaccelerating 
                // calculate the deacceleration needed - ignoring maxAcceleraion
                double t = 2 * remainingDistance / (curentVelocity + targetVelocity);
                double a = (curentVelocity - targetVelocity) / t;
                return Math.max(curentVelocity - a * 0.02, targetVelocity);                    
            }
        } else if(remainingDistance < 0 && targetVelocity <= 0) { // we are moving back - do the reverse caclulation
            return -calculate(-remainingDistance, -curentVelocity, -targetVelocity, maxVelocity, maxAcceleration);
        } else { // we need to move forward, target is behind - return the target velocity 
            return targetVelocity;
        }
    }

    /**
     * calculate the trapezoid velocity - with target velocity 0
     * @param remainingDistance
     * @param curentVelocity
     * @param maxVelocity
     * @param maxAcceleration
     * @return
     */
    public static double calculate(double remainingDistance, double curentVelocity, double maxVelocity, double maxAcceleration) {
        return calculate(remainingDistance, curentVelocity, 0, maxVelocity, maxAcceleration);
    }

    /**
     * calculate the trapezoid velocity to reach the target angle from current angle/position in degrees
     * @param currentAngle - degrees
     * @param targetAngle - degrees
     * @param curentVelocity - degrees per sec
     * @param targetVelocity 
     * @param maxVelocity
     * @param maxAcceleration
     * @return
     */
    public static double calculate(double currentAngle, double targetAngle, double curentVelocity, double targetVelocity, double maxVelocity, double maxAcceleration) {
        double distance = MathUtil.inputModulus(targetAngle - currentAngle,-180, 180);
        return calculate(distance, curentVelocity, targetVelocity, maxVelocity, maxAcceleration);
    }
    public static void main(String[] args) {
        double startTime = System.currentTimeMillis();
        double target = -10000;
        double pos = 0;
        double v = 0;
        for(int i = 0; i < 100000; i++) {
            double tv = calculate(target - pos, v, 0, 2, 4);
            pos += (v + tv) / 2 * 0.02;
            v = tv;
        }
        double endTime = System.currentTimeMillis();
        System.out.printf("pos=%4.2f v = %4.2f\n", pos, v);
        System.out.printf("time = %4.2f\n", (endTime - startTime));

    }
}