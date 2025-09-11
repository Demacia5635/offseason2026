package frc.Demacia.utils.Log;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.Demacia.Geometry.Pose2d;
import frc.Demacia.utils.Log.LogManager.LOG_TARGET;

public class SwerveLogEntry {
    public static void add(String[] moduleNames, SwerveModuleState[] states, SwerveModulePosition[] positions, Pose2d pose, ChassisSpeeds currentSpeed, ChassisSpeeds targetSpeeds) {
        int nModules = states.length;
        // for each module - speed, angle, position and position angle 
        // pose - x, y, angle
        // current speed - vx, vy, omega
        // target speed - vx, vy, omega
        int numSupplieres = 4 * nModules + 3 + 3 + 3;
        LogSupplier[] suppliers = new LogSupplier[numSupplieres];
        int position = 0;
        for(int i = 0; i < 4; i++) {
            SwerveModuleState state = states[i];
            SwerveModulePosition pos = positions[i];
            suppliers[position++] = new LogSupplier(()->state.speedMetersPerSecond, moduleNames[i] + "-spped"  , null);
            suppliers[position++] = new LogSupplier(()->state.angle.getDegrees(), moduleNames[i] + "-angle"  , null);
            suppliers[position++] = new LogSupplier(()->pos.distanceMeters, moduleNames[i] + "-distance"  , null);
            suppliers[position++] = new LogSupplier(()->pos.angle.getDegrees(), moduleNames[i] + "-pos-angle"  , null);
        }
        suppliers[position++] = new LogSupplier(()->pose.getX(), "X"  , null);
        suppliers[position++] = new LogSupplier(()->pose.getY(), "Y"  , null);
        suppliers[position++] = new LogSupplier(()->pose.getRotation().getDegrees(), "heading"  , null);
        suppliers[position++] = new LogSupplier(()->currentSpeed.vxMetersPerSecond, "vx"  , null);
        suppliers[position++] = new LogSupplier(()->currentSpeed.vyMetersPerSecond, "vy"  , null);
        suppliers[position++] = new LogSupplier(()->currentSpeed.omegaRadiansPerSecond, "omega"  , null);
        suppliers[position++] = new LogSupplier(()->targetSpeeds.vxMetersPerSecond, "req-vx"  , null);
        suppliers[position++] = new LogSupplier(()->targetSpeeds.vyMetersPerSecond, "req-vy"  , null);
        suppliers[position++] = new LogSupplier(()->targetSpeeds.omegaRadiansPerSecond, "req-omega"  , null);
        
        new LogEntry(
            "Swerve", 
            suppliers,
            LOG_TARGET.LOG_ONLY, 
            "Swerve", 
            nModules + "Modules", 
            "");

    }

}
