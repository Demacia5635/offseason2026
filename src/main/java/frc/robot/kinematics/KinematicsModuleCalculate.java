package frc.robot.kinematics;

public class KinematicsModuleCalculate {
    public static double calculateVelocity(double alpha, double distance){
        double velocity = (distance * alpha * 50) / Math.sin(alpha);
        return velocity;
    }

    public static double wantedAngle(double wantedAngle){
        return wantedAngle;
    }

    public static double currendAngle(double currentAngle){
        return currentAngle;
    }

    public static double alpaha(double alpha){
        return alpha;
    }

    public static double calculetAngle(double wantedAngle, double currentAngle, double alpha){
        return wantedAngle*alpha + currentAngle+ alpha * 2;
    }
}
