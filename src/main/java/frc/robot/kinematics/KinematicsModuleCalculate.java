package frc.robot.kinematics;

public class KinematicsModuleCalculate {
    public static double calculateVelocity(double alpha, double distance){
        double velocity = (distance * alpha * 50) / Math.sin(alpha);
        return velocity;
    }
}
