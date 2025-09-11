package frc.robot.Drive;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Demacia.utils.Trapezoid;
import frc.Demacia.utils.Utilities;
import frc.robot.RobotContainer;
import frc.robot.Drive.Path.PathPoint;

public class FollowPath extends Command {

    Path path;
    DriveSubsystem drive;
    PathPoint nextPoint = null;
    Pose2d pose;
    ChassisSpeeds speeds;
    double remainingDistance = 0;
    Translation2d toEnd = new Translation2d();
    Translation2d currentVelocity = new Translation2d();
    boolean turning = false;
    double baseHeading = 0;
    double targetHeading;
    

    Trapezoid trpezoid;

    public static final double DISTANCE_ERROR = 0.05;
    public static final double MAX_ALPHA = Math.PI/4;
    public static final double KVelocity = 0.5;
    public static final double KOmega = 0.5;
    public static final double MAX_INITIAL_TURN_ERROR = 0.1;

    public FollowPath(Path path, DriveSubsystem drive) {
        this.path = path;
        this.drive = drive;
        pose = drive.pose;
        speeds = drive.currentChassisSpeeds;
        trpezoid = new Trapezoid(path.maxV, path.maxAcceleration);
        targetHeading = path.endRotation.getRadians();
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        nextPoint = path.first.next;       
        remainingDistance = path.first.remainingDistance;
        turning = true;
    }

    @Override
    public void execute() {
        toEnd.set(nextPoint.startTurn.getX() - pose.getX(), nextPoint.startTurn.getY() - pose.getY());
        double toTargetHeading = toEnd.getAngle().getRadians();
        currentVelocity.set(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double currentV = currentVelocity.getNorm();
        double currentHeading = currentV < 0.1 ? toTargetHeading : currentVelocity.getAngle().getRadians();
        double driveHeadingError = MathUtil.angleModulus(toTargetHeading - currentHeading);
        remainingDistance = toEnd.getNorm();
        double alpha = 0;
        if(turning) {
            if(Math.abs(driveHeadingError) < MAX_INITIAL_TURN_ERROR) {
                turning = false;
                baseHeading = toTargetHeading;
            } else {
                alpha = currentHeading + Utilities.clamp(driveHeadingError, path.maxV / path.radius * RobotContainer.CYCLE_TIME);
            }
        }
        if(!turning) {
            alpha = 2*toTargetHeading - baseHeading;
            if(remainingDistance < DISTANCE_ERROR || Math.abs(alpha) > MAX_ALPHA) {
                // next point 
                nextPoint = nextPoint.next;
                turning = true;
                if(nextPoint != null) {
                    execute();
                    return;
                }
            }
        }
        remainingDistance += nextPoint.remainingDistance;
        double vel = trpezoid.calculate(remainingDistance, currentV, 0);
        double headingError = targetHeading - pose.getRotation().getRadians();
        double omega = Utilities.clamp(headingError*KOmega, path.maxOmega);
        drive.setSpeeds(new ChassisSpeeds(vel*Math.cos(alpha),vel*Math.sin(alpha),omega));
    }

    @Override
    public boolean isFinished() {
        return nextPoint == null;
    }

    @Override
    public void end(boolean interrupted) {
        drive.setSpeeds(new ChassisSpeeds(0,0,0));
    }



}
