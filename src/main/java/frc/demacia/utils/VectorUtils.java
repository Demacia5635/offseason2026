package frc.Demacia.utils;

import frc.Demacia.Geometry.Rotation2d;
import frc.Demacia.Geometry.Translation2d;

public class VectorUtils {

    public static Translation2d intersct(Translation2d p1, Rotation2d v1Angle, Translation2d p2, Rotation2d v2Angle) {
        Translation2d R = new Translation2d(1, v1Angle);
        Translation2d S = new Translation2d(1, v2Angle);
        Translation2d p1ToP2 = p2.minus(p1);
        S.set(S.getY(), -S.getX());
        double t = R.dot(S);
        if(Math.abs(t) > 1e-10) {
            t = p1ToP2.dot(S) / t; 
            return p1.plus(R.timesSelf(t));
        }
        return null;
    }


    public static void main(String[] args) {
        // test the intersect
        // from 1,1 dir 35 and 2,3 dir 55
        Translation2d p1 = new Translation2d(1, 1);
        Translation2d p2 = new Translation2d(2, 3);
        Rotation2d dir1 = Rotation2d.fromDegrees(35);
        Rotation2d dir2 = Rotation2d.fromDegrees(-55);
        double startTime = System.currentTimeMillis()/1000.0;
        Translation2d x = new Translation2d();
        for(int i = 0; i < 1000; i++) {
            p1.set(p1.getX() + 0.1, p1.getY());
            x = intersct(p1, dir1, p2, dir2);
        }
        double endTime = System.currentTimeMillis()/1000.0;
        System.out.println(" avg time = " + (endTime - startTime)*1000.0 + "us");

        Translation2d p1x = x.minus(p1);
        Translation2d p2x = x.minus(p2);
        System.out.printf("p1 = %s, p2 = %s, x=%s\n", p1, p2, x);
        System.out.printf("p1ToP2 = %s\n", p2.minus(p1));
        System.out.printf("p1x = %s, p2x = %s\n", p1x, p2x);
        System.out.printf("dir1 = %s / %s\n", dir1, p1x.getAngle());
        System.out.printf("dir2 = %s / %s\n", dir2, p2x.getAngle());

    }

}
