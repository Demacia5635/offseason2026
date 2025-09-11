package frc.robot.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.Demacia.Geometry.Rotation2d;
import frc.Demacia.Geometry.Translation2d;

public class UdiKinematics1 {

    public static final double DT = 0.02;
    class ModulPos {
        double d;
        double alpha;
        ModulPos(Translation2d pos) {
            d = pos.getNorm();
            alpha = pos.getAngle().getRadians();
        }
    }
    ModulPos[] pos;
    SwerveModuleState[] states;
    double lastHeading = 0;


    UdiKinematics1(Translation2d[] modulsPos) {
        pos = new ModulPos[modulsPos.length];
        states = new SwerveModuleState[pos.length];
        for(int i = 0; i < pos.length; i++) {
            pos[i] = new ModulPos(modulsPos[i]);
            states[i] = new SwerveModuleState(0, 0, new Rotation2d());
        }
    }

    public  SwerveModuleState[] getModuleStates(double heading, ChassisSpeeds speed) {
        double omegat = speed.omegaRadiansPerSecond * DT;
        for(int i = 0; i < pos.length; i++) {
            double alpha = heading + pos[i].alpha + omegat;
            double vx = speed.vxMetersPerSecond - pos[i].d*speed.omegaRadiansPerSecond*(Math.sin(alpha));
            double vy = speed.vyMetersPerSecond + pos[i].d*speed.omegaRadiansPerSecond*(Math.cos(alpha));
            states[i].angle.set(Math.atan2(vy, vx) - heading);
            states[i].speedMetersPerSecond = Math.hypot(vx, vy);
        }
        return states;
    }

    public SwerveModuleState[] states() {
        return states;
    }


    public ChassisSpeeds getChassisSpeeds(SwerveModuleState[] states, double currentHeading) {
        double baseOmega = (currentHeading - lastHeading) / DT;
        // calculate based on module 0/1
        double vx = 0;
        double vy = 0;
        for(int i = 0; i < pos.length; i++) {
            vx += states[i].speedMetersPerSecond * Math.cos(states[i].angle.getRadians() + currentHeading) +
                    baseOmega*pos[i].d*Math.sin(currentHeading + pos[i].alpha);
            vy += states[i].speedMetersPerSecond * Math.sin(states[i].angle.getRadians() + currentHeading) - 
                    baseOmega*pos[i].d*Math.cos(currentHeading + pos[i].alpha);
        }
        lastHeading = currentHeading;
        return new ChassisSpeeds(vx/pos.length, vy/pos.length, baseOmega);
    }

    private  static edu.wpi.first.math.kinematics.SwerveModuleState[] baseKinematics(SwerveDriveKinematics kinematics, double heading, ChassisSpeeds speed) {
        Rotation2d rot = new Rotation2d(heading);
        ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(speed, rot);
        return kinematics.toSwerveModuleStates(robotRelative);
    }

    private static void compare(double heading, ChassisSpeeds speed, edu.wpi.first.math.kinematics.SwerveModuleState[] baseStates, SwerveModuleState[] states) {
        System.out.printf("compare for - heading = %5.1f, vx=%4.2f vy=%4.2f omega=%5.2f\n", heading, speed.vxMetersPerSecond, speed.vyMetersPerSecond, Math.toDegrees(speed.omegaRadiansPerSecond));
        for(int i = 0; i < baseStates.length; i++) {
            System.out.printf("   %d: v = %5.3f [%4.2f, %4.2f]  a = %6.4f [%5.3f, %5.3f]\n", i, 
                states[i].speedMetersPerSecond - baseStates[i].speedMetersPerSecond, states[i].speedMetersPerSecond, baseStates[i].speedMetersPerSecond,
                states[i].angle.getDegrees()-baseStates[i].angle.getDegrees(), states[i].angle.getDegrees(), baseStates[i].angle.getDegrees());
        }
    }

    public static final double X = 0.35;
    public static final double Y = 0.3;

    public static void main(String[] args) {

        Translation2d[] modulePositionOnRobot = {
            new Translation2d(X,Y), new Translation2d(X,-Y), new Translation2d(-X,Y), new Translation2d(-X,-Y)};
        UdiKinematics1 u = new UdiKinematics1(modulePositionOnRobot);
        SwerveDriveKinematics  kinematics = new SwerveDriveKinematics(modulePositionOnRobot);
        edu.wpi.first.math.kinematics.SwerveModuleState[] baseStates;

        ChassisSpeeds s = new ChassisSpeeds(2,0,1);
        double heading = 0;
        u.getModuleStates(heading, s);
        baseStates = baseKinematics(kinematics, heading, s);
        compare(heading, s, baseStates, u.states);
        System.out.println(" calculated = " + u.getChassisSpeeds(u.states, heading + s.omegaRadiansPerSecond * DT));
        s.vxMetersPerSecond = 1;
        s.vyMetersPerSecond = 1;
        s.omegaRadiansPerSecond = -1;
        u.getModuleStates(0, s);
        baseStates = baseKinematics(kinematics, heading, s);
        compare(heading, s, baseStates, u.states);

        System.out.println(" calculated = " + u.getChassisSpeeds(u.states, heading + s.omegaRadiansPerSecond * DT));
        double startTime1 = System.currentTimeMillis();
        for(int i = 0; i < 1000; i++) {
            u.getModuleStates(heading, s);
        }
        double endTime1 = System.currentTimeMillis();
        double startTime2 = System.currentTimeMillis();
        for(int i = 0; i < 1000; i++) {
            baseStates = baseKinematics(kinematics, heading, s);
        }
        double endTime2 = System.currentTimeMillis();

        System.out.println("time1 = " + (endTime1 - startTime1) + "us");
        System.out.println("time2 = " + (endTime2 - startTime2) + "us");

    }



}
