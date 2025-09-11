package frc.robot.Drive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Demacia.Geometry.Pose2d;
import frc.Demacia.Geometry.Rotation2d;
import frc.Demacia.Geometry.Translation2d;

public class Path {


    class PathPoint {
        Translation2d point;
        Translation2d center;
        Translation2d startTurn;
        Translation2d endTurn;
        Translation2d fromPrev = null;
        Translation2d toCenter;
        boolean positiveTurn = true;
        double radius;
        double remainingDistance = 0;

        PathPoint prev = null;
        PathPoint next = null;

        PathPoint(Translation2d point, PathPoint prev, double radius) {
            this.point = point;
            this.radius = radius;
            setPrev(prev);

        }

        void setPrev(PathPoint prev) {
            this.prev = prev;
            if(prev == null) {
                fromPrev = null;
            } else {
                fromPrev = point.minus(prev.point);
                prev.next = this;
            }
        }


        void calculateCenter() {
            double fromAngle = fromPrev.getAngle().getRadians();
            double toAngle = next.fromPrev.getAngle().getRadians();
            double angle = MathUtil.angleModulus(toAngle - fromAngle);
            positiveTurn = angle > 0;
            if(positiveTurn) {
                angle = fromAngle + angle + (Math.PI - angle)/2;
            } else {
                angle = fromAngle + angle - (Math.PI + angle)/2;
            }
            center = new Translation2d(point.getX() + radius * Math.cos(angle), point.getY() + radius * Math.sin(angle));
            toCenter = center.minus(prev.center);
        }

        void calculateTurnPoints() { // calculate from end to start - calculate start point and prev end point
            double angleFromLast = toCenter.getAngle().getRadians();
            double length = toCenter.getNorm();
            if(radius == 0) { // last point
                double offset = Math.acos(radius / length);
                double angle = angleFromLast + (prev.positiveTurn ? -offset : offset);
                Translation2d t = new Translation2d(prev.radius, new Rotation2d(angle));
                prev.endTurn = prev.center.plus(t);
            } else if(prev.radius == 0) { // first 
                double offset = Math.acos(radius / length);
                double angle = -angleFromLast + (positiveTurn ? -offset : offset);
                Translation2d t = new Translation2d(radius, new Rotation2d(angle));
                startTurn = center.plus(t);
            } else if(prev.positiveTurn == positiveTurn) { // mid point - same rotation
                double offset = Math.PI/2;
                double angle = angleFromLast + (positiveTurn ? -offset: offset);
                Translation2d t = new Translation2d(radius, new Rotation2d(angle));
                startTurn = center.plus(t);
                prev.endTurn = prev.center.plus(t);
            } else { // mid point = reverse turn
                double offset = Math.acos(radius / length * 2);
                double angle = angleFromLast + (positiveTurn ? offset : -offset);
                Translation2d t = new Translation2d(radius, new Rotation2d(angle));
                prev.endTurn = prev.center.plus(t);
                startTurn = center.minus(t);
            }
        }

        void calculateDistance() {
            if(prev == null) { // first
                remainingDistance = next.remainingDistance + point.getDistance(next.startTurn);
            } else if(next == null) {
                remainingDistance = 0;
            } else {
                remainingDistance = next.remainingDistance + 2*radius*Math.asin(startTurn.getDistance(endTurn) / radius / 2) + endTurn.getDistance(next.startTurn);
            }

        }

        @Override
        public String toString() {
            return String.format("point=%s center=%s start=%s end=%s from=%s to=%s positive=%b radius=%f", point, center, startTurn, endTurn, fromPrev, toCenter, positiveTurn, radius);
        }
    }

    Rotation2d startRotation;
    Rotation2d endRotation;

    PathPoint first;
    PathPoint last;
    double radius;
    double maxV;
    double maxOmega;
    double maxTurnRate;
    double maxAcceleration;
    double maxCentrifugalG;
    /**
     * 
     * @param startPose
     * @param endPose
     * @param midPoints
     * @param radius
     * @param maxV
     * @param maxOmega
     * @param maxTurnRate
     * @param maxAcceleration
     */
    public Path(Pose2d startPose, Pose2d endPose, Translation2d[] midPoints, double radius, double maxV,
            double maxOmega, double maxTurnRate, double maxAcceleration) {
        startRotation = startPose.getRotation();
        endRotation = endPose.getRotation();
        first = new PathPoint(startPose.getTranslation(), null, 0);
        first.center = first.point;
        first.startTurn = first.point;
        first.endTurn = first.point;
        last = new PathPoint(endPose.getTranslation(), null, 0);
        last.center = last.point;
        last.startTurn = last.point;
        last.endTurn = last.point;
        first.next = last;
        last.prev = first;
        for(Translation2d t : midPoints) {
            PathPoint p = new PathPoint(t, last.prev, radius);
            p.next = last;
            last.prev = p;
        }
        last.setPrev(last.prev);

        this.radius = radius;
        this.maxV = maxV;
        this.maxOmega = maxOmega;
        this.maxTurnRate = maxTurnRate;
        this.maxAcceleration = maxAcceleration;
       calculateTurnCenters();
        calculateTurnPoints();
        calculateDistances();
        Field2d fld = new Field2d();
        fld.setRobotPose(startPose);
        SmartDashboard.putData("path", fld);

//        fld.getObject("Points").setPoses(getPoints(Rotation2d.kZero));
        //fld.getObject("Centers").setPoses(getCenters(Rotation2d.kCCW_90deg));
        fld.getObject("traj").setTrajectory(getTrajectory());

    }

    private void calculateTurnCenters() {
        for(PathPoint p = first.next; p != last; p = p.next) {
            p.calculateCenter();
        }
        last.toCenter = last.center.minus(last.prev.center);
    }
    private void calculateDistances() {
        for(PathPoint p = last; p != null; p = p.prev) {
            p.calculateDistance();
        }
        last.toCenter = last.center.minus(last.prev.center);
    }


    private void calculateTurnPoints() {
        for(PathPoint p = last; p != first; p = p.prev) {
            p.calculateTurnPoints();
        }
    }

    public List<Pose2d> getCenters(Rotation2d rot) {
        ArrayList<Pose2d> l = new ArrayList<>();
        for(PathPoint p = first; p != null; p = p.next) {
            l.add(new Pose2d(p.center, rot));
        }
        return l;
    } 
    public List<Pose2d> getPoints(Rotation2d rot) {
        ArrayList<Pose2d> l = new ArrayList<>();
        for(PathPoint p = first; p != null; p = p.next) {
            l.add(new Pose2d(p.point, rot));
        }
        return l;
    } 
    public List<Pose2d> getTurnPoints(Rotation2d rot) {
        ArrayList<Pose2d> l = new ArrayList<>();
        for(PathPoint p = first; p != null; p = p.next) {
            l.add(new Pose2d(p.startTurn, rot));
            if(p.endTurn != p.startTurn) 
                l.add(new Pose2d(p.endTurn, rot));
        }
        return l;
    } 

    void print() {
        for(PathPoint p = first; p != null; p = p.next) {
            System.out.println(p.toString());
        }
    }

    Trajectory getTrajectory() {
        ArrayList<State> l = new ArrayList<>();
        double time = 0;
        l.add(new State(time, maxV, 0, new Pose2d(first.point, startRotation),0));
        for(PathPoint p = first.next; p != last; p = p.next) {
            time += p.prev.endTurn.getDistance(p.startTurn) / maxV;
            l.add(new State(time, maxV, 0, new Pose2d(p.startTurn, endRotation), maxV/radius));
            time += p.endTurn.getDistance(p.startTurn) / maxV;
            l.add(new State(time, maxV, 0, new Pose2d(p.endTurn, endRotation), 0));
        }
        time += last.prev.endTurn.getDistance(last.point) / maxV;
        l.add(new State(time, 0, 0, new Pose2d(last.point, endRotation), 0));
        return new Trajectory(l);
    }
}
