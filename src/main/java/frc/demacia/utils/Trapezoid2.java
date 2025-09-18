package frc.Demacia.utils;


public class Trapezoid2 {
    public static double calculate(double remainingDistance, double curentVelocity, double targetVelocity, double maxVelocity, double maxAcceleration) {
        if(remainingDistance > 0 && targetVelocity >= 0) {
            // calculate the maximum velocity if continuing acceleration/deacceleratin regardless of maxVelocity
            double maxV = Math.sqrt(remainingDistance*maxAcceleration + curentVelocity*curentVelocity + targetVelocity * targetVelocity);
            // define the max velocity between the calculated on and the provided one
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