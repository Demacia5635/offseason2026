package frc.robot.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class ICRData {

    public enum ICRType { RADIAL, LINEAR, NO_MOVE, SLIP};

    public static final double MIN_OMEGA_DIF = 0.1;
    /**
     *
     */
    // the ICR data
    Translation2d ICR = new Translation2d();
    double omega1 = 0;
    double omega2 = 0;
    ICRType type;

    // v1/v2 as POSE
    Translation2d p1;
    Translation2d p2;
    Translation2d p1Top2;
    double p1ToP2Distance;
    Rotation2d p1ToP2Dir;
    SwerveModuleState v1;
    SwerveModuleState v2;
    Translation2d R = new Translation2d();
    Translation2d S = new Translation2d();

    ICRData(Translation2d module1Position, Translation2d module2Position, SwerveModuleState module1Velocity, SwerveModuleState module2Velocity) {
        p1 = module1Position;
        p2 = module2Position;
        p1Top2 = p2.minus(p1);
        p1ToP2Distance = p1Top2.getNorm();
        p1ToP2Dir = p1Top2.getAngle();
        v1 = module1Velocity;
        v2 = module2Velocity;
    }

    void calculateICR() {
        type = ICRType.RADIAL;
        if(v1.speedMetersPerSecond == 0 || v2.speedMetersPerSecond == 0) { // at least one module is stationary
            if(v1.speedMetersPerSecond == v2.speedMetersPerSecond) { // not moving
                type = ICRType.NO_MOVE;
                ICR.set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
                omega1 = 0;
                omega2 = 0;
            } else if(v1.speedMetersPerSecond == 0) { // module 1 stationary - 2 moving
                ICR.set(p1.getX(),p1.getY());
                double angle = MathUtil.inputModulus(v2.angle.getRadians() - p1ToP2Dir.getRadians(), -Math.PI, Math.PI);
                omega2 = v2.speedMetersPerSecond / p1ToP2Distance * Math.signum(angle);
                omega1 = omega2;
                if(Math.PI-Math.abs(angle) > MIN_OMEGA_DIF) { // direction is not OK
                    type = ICRType.SLIP;
                }
            } else {
                ICR.set(p2.getX(), p2.getY());
                double angle = -MathUtil.inputModulus(v2.angle.getRadians() - p1ToP2Dir.getRadians(), -Math.PI, Math.PI);
                omega1 = v1.speedMetersPerSecond / p1ToP2Distance * Math.signum(angle);
                omega2 = omega1;
                if(Math.PI - Math.abs(v1.angle.getRadians() - p1ToP2Dir.getRadians()) %  Math.PI > MIN_OMEGA_DIF) { // direction is OK
                    type = ICRType.SLIP;
                }
            }
            return;
        }

        R.set(v1.angle.getCos(), v1.angle.getSin());
        S.set(v2.angle.getSin(), -v2.angle.getCos());
        double t = R.dot(S);
        if(Math.abs(t) > 1e-10) {
            t = p1Top2.dot(S) / t; 
            R.timesSelf(t);
            ICR.set(p1.getX() + R.getX(), p2.getY() + R.getY());
            omega1 = v1.speedMetersPerSecond / Math.hypot(ICR.getX() -p1.getX(), ICR.getY() - p1.getY() * Math.signum(t));
            omega2 = v2.speedMetersPerSecond / Math.hypot(ICR.getX() -p2.getX(), ICR.getY() - p2.getY() * Math.signum(t));
            if(Math.abs(omega1 - omega2) < MIN_OMEGA_DIF) {
                type = ICRType.SLIP;
            }
        } else { // linear move
            type = ICRType.LINEAR;
            omega1 = 0;
            omega2 = 0;
            ICR.set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }                
    }
}