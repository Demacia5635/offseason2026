package frc.Demacia.utils;


public class Trapezoid2 {
    double maxVelocity; // Maximum permissible velocity
    double maxAcceleration; // Maximum permissible acceleration
    double startVelocity;
    double endVelocity;
    double distance;
    double startDeaccelraionDistance = 0;
    double endAccelerationDistance = 0;
    double dir = 1;


    // Constructor to initialize with maximum velocity and acceleration
    public Trapezoid2(double maxVelocity, double maxAcceleration, double startVelocity, double endVelocity, double distance) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        if(distance > 0) {
            this.startVelocity = startVelocity;
            this.endVelocity = endVelocity;
            this.distance = distance;
            dir = 1;
        } else {
            this.startVelocity = -startVelocity;
            this.endVelocity = -endVelocity;
            this.distance = -distance;
            dir = -1;

        }
        calculateDistances();

    }

    private void calculateDistances() {
        double deaccelrateTime = (maxVelocity - endVelocity) / maxAcceleration;
        startDeaccelraionDistance = (endVelocity + maxVelocity) * deaccelrateTime / 2;
        double accelrateTime = (maxVelocity - startVelocity) / maxAcceleration;
        double accelerateDistance = (maxVelocity + startVelocity) * accelrateTime / 2;
        if(distance >= startDeaccelraionDistance + accelerateDistance) {
            endAccelerationDistance = distance - accelerateDistance;
        } else {
            double maxV = Math.sqrt(distance * maxAcceleration + startVelocity*startVelocity + endVelocity * endVelocity);
            deaccelrateTime = (maxV - endVelocity) / maxAcceleration;
            startDeaccelraionDistance = (endVelocity + maxV) * deaccelrateTime / 2;
            endAccelerationDistance = startDeaccelraionDistance;
        }
    }


    // Function to calculate the next velocity setpoint, based on remaining distance and current and target velocities
    public double calculate(double remainingDistance, double curentVelocity) {
        double d = remainingDistance * dir;
        double v = curentVelocity * dir;
        if(d > startDeaccelraionDistance) { // accelerate
            return (Math.min(maxVelocity, v + maxAcceleration * 0.02))*dir;
        } else {
            double t = 2 * d / (v + endVelocity);
            double a = (v - endVelocity) / t;
            return Math.max(v - a * 0.02, endVelocity) * dir;
        }        
    }

    @Override
    public String toString() {
        return String.format("Trapezoid distance %4.2f from %4.2f to %4.2f - accel to %4.2f, keepMax to %4.2f\n", distance, startVelocity, endVelocity, endAccelerationDistance, startDeaccelraionDistance);
    }

    public static void main(String[] args) {
        double v = 0;
        double remain = -4;
        Trapezoid2 t = new Trapezoid2(3, 6, v, 0, remain);
        int n = 0;
        while(Math.abs(remain) > 0.03) {
            double nv = t.calculate(remain, v);
            remain -= (v+nv)/2*0.02;
            v = nv;
            System.out.printf("v = %4.2f r = %5.3f\n",v,remain);
            n++;
        }
        System.out.println(" time = " + 0.02*n);
        
    }
}