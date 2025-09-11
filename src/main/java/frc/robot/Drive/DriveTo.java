package frc.robot.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Demacia.utils.Utilities;

public class DriveTo extends Command {
    double x;
    double y;
    double v;
    double maxOmega;
    DriveSubsystem drive;
    boolean isFinal;
    Pose2d pose;

    double remainingDistance = 0;
    double initialHeading = Double.MAX_VALUE;
    Translation2d toEnd = new Translation2d(0,0);
    Rotation2d currentDriveRotation = new Rotation2d();
    ChassisSpeeds currentSpeeds;
    double turnRate;
    boolean turnedToTarget = false;
    double targetHeading;
    boolean targetHeadingReal;


    public static final double NON_FINAL_DISTANCE_ERROR = 0.5;
    public static final double FINAL_DISTANCE_ERROR = 0.05;
    public static final double KVelocity = 0.5;
    public static final double KOmega = 0.5;
    public static final double MAX_INITIAL_TURN_ERROR = 0.1;


    public DriveTo(double x, double y, double heading, double v, double maxOmega, double maxTurnRate, DriveSubsystem drive, boolean isFinal) {
        this.x = x;
        this.y = y;
        this.v = v;
        this.maxOmega = maxOmega;
        this.drive = drive;
        this.isFinal = isFinal;
        this.turnRate = Math.toRadians(maxTurnRate) * 0.02;
        pose = drive.pose;
        this.targetHeading = Math.toRadians(heading);
        targetHeadingReal = Math.abs(heading) < Math.PI;
        currentSpeeds = drive.currentChassisSpeeds;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        toEnd.set(x - pose.getX(), y - pose.getY());
        remainingDistance = toEnd.getNorm();
        turnedToTarget = false;

    }

    @Override
    public void execute() {
        toEnd.set(x - pose.getX(), y - pose.getY());
        currentDriveRotation.set(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        double currentHeading = currentDriveRotation.getRadians();
        double toTargetHeading = toEnd.getAngle().getRadians();
        double driveHeadingError = MathUtil.angleModulus(toTargetHeading - currentHeading);
        remainingDistance = toEnd.getNorm();
        
        double alpha = 0;
        if(!turnedToTarget) {
            if(Math.abs(driveHeadingError) < MAX_INITIAL_TURN_ERROR) {
                turnedToTarget = true;
                initialHeading = toTargetHeading;
            } else {
                alpha = currentHeading + Utilities.clamp(driveHeadingError, -turnRate, turnRate);
            }
        }
        if(turnedToTarget) {
            alpha = 2*toTargetHeading - initialHeading;
        }
        double vel = isFinal ? Math.min(remainingDistance * KVelocity, v) : v;
        double headingError = targetHeading - pose.getRotation().getRadians();
        if(targetHeadingReal) {
            headingError = MathUtil.angleModulus(headingError);
        }
        double omega = Utilities.clamp(headingError*KOmega, maxOmega);
        drive.setSpeeds(new ChassisSpeeds(vel*Math.cos(alpha),vel*Math.sin(alpha),omega));
    } 

    @Override
    public boolean isFinished() {
        return remainingDistance < (isFinal ? FINAL_DISTANCE_ERROR : NON_FINAL_DISTANCE_ERROR);
    }

    @Override
    public void end(boolean interrupted) {
        if(isFinal)
            drive.setSpeeds(new ChassisSpeeds(0,0,0));
    }    

}
