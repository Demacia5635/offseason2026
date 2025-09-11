package frc.robot.Drive;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModulePosition extends  edu.wpi.first.math.kinematics.SwerveModulePosition {
    public Rotation2d angle = new Rotation2d();

    public SwerveModulePosition(double distance, Rotation2d angle) {
        super(distance, angle);
        this.angle = angle;
    }


}
