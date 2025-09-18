package frc.robot.kinematics;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
public class KinematicsModuleCalculate {

    Translation2d FRONT_LEFT = new Translation2d(+0.381, +0.381);
    Translation2d FRONT_RIGHT = new Translation2d(+0.381, -0.381);  
    Translation2d BACK_LEFT = new Translation2d(-0.381, +0.381);
    Translation2d BACK_RIGHT = new Translation2d(-0.381, -0.381);

  
    public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    );


    ChaxssisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    );


    double velocityX = chassisSpeeds.vxMetersPerSecond;
    double velocityY = chassisSpeeds.vyMetersPerSecond;
    double omega     = chassisSpeeds.omegaRadiansPerSecon;

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
