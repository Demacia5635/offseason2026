package frc.robot.kinematics;

public class KinematicsModuleCalculate {
    public static double calculateVelocity(double alpha, double distance){
        double velocity = (distance * alpha * 50) / Math.sin(alpha);
        return velocity;
    }

    
    public double position(double velocityX, double velocityY, double omega, double position){
        position = (velocityX+velocityY+omega) * 0.02;
        return position;
    }

    public static double calculateAngle(double wantedAngle, double currentAngle, double alpha){
        return wantedAngle + currentAngle+ alpha * 2;
    }

}
