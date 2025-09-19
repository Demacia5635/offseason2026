package frc.robot.kinematics;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
public class KinematicsModuleCalculate {

    public double calculateVelocity(double alpha, double distance){
        double velocity = (distance * alpha * 50) / Math.sin(alpha);
        return velocity;
    }

    
    public double position(double velocityX, double velocityY, double omega, double position){
        position = (velocityX+velocityY+omega) * 0.02;
        return position;
    }

    public double calculateAngle(double wantedAngle, double currentAngle, double alpha){
        return wantedAngle + currentAngle+ alpha * 2;
    }

}
