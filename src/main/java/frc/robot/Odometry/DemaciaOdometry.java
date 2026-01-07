// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Odometry;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Add your docs here. */
public class DemaciaOdometry {
    private Pose2d pose;
    private final Translation2d[] modulePositions;
    private SwerveModulePosition[] lastPositions;
    private Rotation2d lastAngle = Rotation2d.kZero;
    // private final AccelOdometry accelOdometry;
    private boolean isSkidding = false; // need to add logic for skidding detection, and what to do when skidding
    private static DemaciaOdometry instance;
    private final double modulesDistanceSum;

    // private double accelParameter = 0.3;

    public DemaciaOdometry(Translation2d[] modulePositions) {
        this.modulePositions = modulePositions;
        this.lastPositions = new SwerveModulePosition[modulePositions.length];

        for (int i = 0; i < lastPositions.length; i++) {
            lastPositions[i] = new SwerveModulePosition();
        }
        // this.accelOdometry = new AccelOdometry();
        this.pose = new Pose2d();
        modulesDistanceSum = Arrays.stream(modulePositions).mapToDouble(pos -> pos.getNorm()).sum();
    }

    public static DemaciaOdometry getOdometryInstance(Translation2d[] modulePositions) {
        if (instance == null) instance = new DemaciaOdometry(modulePositions);
        return instance;
    }

    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] currentPositions) {
        Translation2d[] moduleDisplacements = new Translation2d[modulePositions.length];
        for (int i = 0; i < modulePositions.length; i++) {
            moduleDisplacements[i] = calculateModuleDisplacement(lastPositions[i], currentPositions[i]);

        }

        Twist2d robotDisplacement = calculateRobotDisplacement(moduleDisplacements);
        robotDisplacement.dtheta = gyroAngle.minus(lastAngle).getRadians();

        pose = pose.exp(robotDisplacement);

        lastPositions = currentPositions;
        lastAngle = gyroAngle;
        return new Pose2d(pose.getTranslation(), gyroAngle);

    }

    private Translation2d calculateModuleDisplacement(SwerveModulePosition lastPosition,
            SwerveModulePosition currentPosition) {
        double arcLength = currentPosition.distanceMeters - lastPosition.distanceMeters;
        Rotation2d deltaAlpha = currentPosition.angle.minus(lastPosition.angle);
        if (Math.abs(deltaAlpha.getDegrees()) < 1E-9) {
            return new Translation2d(arcLength, currentPosition.angle);
        } else {
            Rotation2d centralAngle = deltaAlpha;
            double radius = arcLength / centralAngle.getRadians();
            double chordLength = 2 * radius * Math.sin(centralAngle.getRadians() / 2);
            Rotation2d chordAngle = lastPosition.angle.plus(centralAngle.times(0.5));

            return new Translation2d(chordLength, chordAngle);
        }
    }

    private Twist2d calculateRobotDisplacement(Translation2d[] moduleDisplacements) {
        double x = 0;
        double y = 0;
        int i = 0;
        for (Translation2d moduleDisplacement : moduleDisplacements) {
            x += moduleDisplacement.getX() * (modulePositions[i].getNorm() / modulesDistanceSum);
            y += moduleDisplacement.getY() * (modulePositions[i].getNorm() / modulesDistanceSum);
            i++;
        }
        return new Twist2d(x, y, 0);
    }

    public void resetPose() {
        resetPose(new Pose2d());
    }

    public void resetPose(Pose2d pose) {
        this.pose = pose;
        lastAngle = pose.getRotation();
    }

    public Pose2d getPose2d() {
        return this.pose;
    }

}
