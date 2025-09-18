package frc.robot.kinematics;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModuleState;

public class KinematicsModuleCalculate {
    private double calculateVelocity(double initialVelocity, double distance, double alpha){
        double velocity = (distance * alpha * 50) / Math.sin(alpha);
        double targetVelocity = 100 * distance - initialVelocity;
        return targetVelocity;
    }

    private double calculateAngle(double currentAngle, double alpha){
        return currentAngle + alpha * 2;
    }

    public static ModuleState[] toModuleStates(){
        return new ModuleState[4];
    }
}
