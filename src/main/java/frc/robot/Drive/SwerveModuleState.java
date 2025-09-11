package frc.robot.Drive;

import frc.Demacia.Geometry.Rotation2d;

public class SwerveModuleState extends edu.wpi.first.math.kinematics.SwerveModuleState {
    public double distanceMeters;
    public Rotation2d angle;

    public SwerveModuleState(double metersPerSecond, double distanceMeters, Rotation2d angle) {
        super();
        this.speedMetersPerSecond = metersPerSecond;
        this.angle = angle;
        this.distanceMeters = distanceMeters;

    }
    public SwerveModuleState(double metersPerSecond, double distanceMeters, double angleRadians) {
        this(metersPerSecond, distanceMeters, new Rotation2d(angleRadians));        
    }

    public void copy(SwerveModuleState other) {
        this.speedMetersPerSecond = other.speedMetersPerSecond;
        this.distanceMeters = other.distanceMeters;
        this.angle.set(other.angle.getRadians());
    }

}
