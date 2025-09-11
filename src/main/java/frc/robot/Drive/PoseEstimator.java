package frc.robot.Drive;

import frc.Demacia.Geometry.Pose2d;
import frc.Demacia.Geometry.Translation2d;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;

public class PoseEstimator {

    private ChassisSpeeds lastSpeeds;
    private ChassisSpeeds currentSpeeds;
    private StatusSignal<Angle> timeSignal;
    private StatusSignal<Angle> gyroSignal;
    private double lastTime;
    private double lastHeading;
    private double gyroOffset;
    private Pose2d pose;

    private static final double MAX_BUFFER_TIME = 1.5;

    class PositionCorrection {
        double deltaX;
        double deltaY;
        double deltaHeading;
        double time;

        PositionCorrection next = null;
        PositionCorrection prev = null;

        PositionCorrection(double deltaX, double deltaY, double deltaHeading, double time) {
            this.deltaX = deltaX;
            this.deltaY = deltaY;
            this.deltaHeading = deltaHeading;
            this.time = time;
            addToList();
        }

        private void addToList() {
            if(last == null) {
                last = this;
            } else if(time > last.time - MAX_BUFFER_TIME) {
                PositionCorrection p = last;
                while(p.time > time)
                    p = p.prev;
                prev = p;
                next = p.next;
                p.next = this;
                if(next == null) {
                    last = this;
                } else {
                    next.prev = this;
                }
            }           
        }
        private void removeFromList() {
            prev.next = next;
            next.prev = prev;
        }
    }

    PositionCorrection last = null;
    PositionCorrection first = new PositionCorrection(0,0,0,0);

    public PoseEstimator(ChassisSpeeds currentSpeeds, StatusSignal<Angle> timeSignal,StatusSignal<Angle> gyroSignal, Pose2d pose) {
        this.currentSpeeds = currentSpeeds;
        lastSpeeds = new ChassisSpeeds(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond, currentSpeeds.omegaRadiansPerSecond);
        this.timeSignal = timeSignal;
        this.gyroSignal = gyroSignal;
        lastTime = timeSignal.getTimestamp().getTime();
        lastHeading = pose.getRotation().getDegrees();
        gyroOffset = lastHeading - gyroSignal.getValue().in(Degrees);
        this.pose = pose;
    }

    public void setPose() {
        lastHeading = pose.getRotation().getDegrees();
        gyroOffset = lastHeading - gyroSignal.getValue().in(Degrees);
    }

    public void updatePose() {
        double currentTime = timeSignal.getTimestamp().getTime();
        double deltaTime = currentTime - lastTime;
        double heading = gyroSignal.getValue().in(Degrees) + gyroOffset;
        PositionCorrection p = new PositionCorrection(
            (currentSpeeds.vxMetersPerSecond + lastSpeeds.vxMetersPerSecond)*deltaTime/2,
            (currentSpeeds.vyMetersPerSecond + lastSpeeds.vyMetersPerSecond)*deltaTime/2,
            heading - lastHeading,
            currentTime);
        cleanList();
        pose.getTranslation().set(pose.getX() + p.deltaX, pose.getY() + p.deltaY);
        pose.getRotation().setDegrees(heading);
        lastTime = currentTime;
        lastSpeeds.vxMetersPerSecond = currentSpeeds.vxMetersPerSecond;
        lastSpeeds.vyMetersPerSecond = currentSpeeds.vyMetersPerSecond;
        lastSpeeds.omegaRadiansPerSecond = currentSpeeds.omegaRadiansPerSecond;
        lastHeading = heading;
    }

    Translation2d getAccumulatedCorrection(double time) {
        Translation2d t = new Translation2d();
        PositionCorrection p = first.next;
        while(p != null && p.time < time) {
            p = p.next;
        }
        if(p == null) {
            return t;
        }
        // calculate interpulated from prev to p 
        double delta = p.time - time;
        double ratio = delta/(p.time - p.prev.time);
        t.set(p.deltaX*ratio, p.deltaY*ratio);
        for(p = p.next; p != null; p = p.next) {
            t.set(t.getX() + p.deltaX, t.getY() + p.deltaY);
        }
        return t;
    }

    public void updateVision(Pose2d visionPose, double time) {
        if(time > last.time - MAX_BUFFER_TIME) {
            Translation2d t = getAccumulatedCorrection(time);
            Translation2d t1 = pose.getTranslation().minus(t);
            Translation2d correction = visionPose.getTranslation().minus(t);
            correction.timesSelf(0.5);
            t1.plusSelf(correction).plusSelf(t);
            pose.getTranslation().set(t1.getX(), t1.getY());
        }
    }

    void cleanAll() {
        first.next = null;
        last = first;
    }
    void cleanList() {
        while(first.next != null && first.next.time < last.time - MAX_BUFFER_TIME) {
            first.next.removeFromList();
        }
    }
}
