package frc.robot.kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Translation2d;

public class data {
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

    SwerveDriveKinematics[] modol={
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_LEFT
    };

    double velocityX = chassisSpeeds.vxMetersPerSecond;
    double velocityY = chassisSpeeds.vyMetersPerSecond;
    double omega     = chassisSpeeds.omegaRadiansPerSecon;
    
}
