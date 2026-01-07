// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Odometry;

import java.util.NavigableMap;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.TreeMap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class DemaciaPoseEstimator {

    private final double MAX_HISTORY_SECONDS = 1.5;

    private Matrix<N3, N1> stateSTD;
    private Matrix<N3, N1> visionSTD;

    private DemaciaOdometry odometry;
    private Pose2d odometryPose;
    private Pose2d estimatedPose;

    private final TimeInterpolatableBuffer<Pose2d> odometryBuffer;

    private NavigableMap<Double, VisionUpdate> visionUpdates = new TreeMap<>();

    private final Matrix<N3, N1> stateStdSquared = new Matrix<>(Nat.N3(), Nat.N1());
    private final Matrix<N3, N3> kalmanMatrix = new Matrix<>(Nat.N3(), Nat.N3());

    public DemaciaPoseEstimator(Translation2d[] modulePositions, Matrix<N3, N1> stateSTD, Matrix<N3, N1> visionSTD) {

        this.odometry = new DemaciaOdometry(modulePositions);
        this.stateSTD = stateSTD;
        this.visionSTD = visionSTD;
        this.odometryBuffer = TimeInterpolatableBuffer.createBuffer(MAX_HISTORY_SECONDS);

        for (int i = 0; i < 3; ++i) {
            stateStdSquared.set(i, 0, stateSTD.get(i, 0) * stateSTD.get(i, 0));
        }

        this.estimatedPose = Pose2d.kZero;
        this.odometryPose = Pose2d.kZero;
    }

    public void updateStateSTD(Matrix<N3, N1> stateSTD) {
        this.stateSTD = stateSTD;
    }

    public void updateVisionSTD(Matrix<N3, N1> visionSTD) {
        this.visionSTD = visionSTD;

    }

    public void updateVisionAndStateSTD(Matrix<N3, N1> stateSTD, Matrix<N3, N1> visionSTD) {
        updateStateSTD(stateSTD);
        updateVisionSTD(visionSTD);
    }

    public void addOdomteryCalculation(OdometryObservation odometryCalculation, Translation2d currentVelocity) {
        var odometryEstimation = odometry.update(odometryCalculation.gyroAngle(), odometryCalculation.swerveModules());
        odometryBuffer.addSample(odometryCalculation.timeStamp(), odometryPose);

        if (visionUpdates.isEmpty()) {
            estimatedPose = odometryEstimation;
        } else {
            var visionUpdate = visionUpdates.get(visionUpdates.lastKey());
            estimatedPose = visionUpdate.compensate(odometryEstimation);
        }

    }

    public void addVisionMeasurement(VisionMeasurment visionMeasurment) {

        try {
            if (odometryBuffer.getInternalBuffer().lastKey() - visionMeasurment.timeStamp > MAX_HISTORY_SECONDS
                    || odometryBuffer.getInternalBuffer().isEmpty()) { // case for measurment too old
                return;
            }
        } catch (NoSuchElementException e) {
            return;
        }

        var sample = odometryBuffer.getSample(visionMeasurment.timeStamp());
        if (sample.isEmpty())
            return; // case for no sample

        updateKalmanFilter();

        boolean is2DPose = visionMeasurment.angle.isPresent();

        Transform2d currentPoseToSample = estimatedPose.minus(sample.get());
        Pose2d estimateAtTime = sample.get().plus(currentPoseToSample);
        Transform2d estimateAndVisionAtTimeDiff = new Transform2d(estimateAtTime,
                new Pose2d(visionMeasurment.pos,
                        is2DPose ? estimateAtTime.getRotation() : visionMeasurment.angle.get()));
        var kalmanTimesDiff = kalmanMatrix.times(
                VecBuilder.fill(estimateAndVisionAtTimeDiff.getX(), estimateAndVisionAtTimeDiff.getY(),
                        visionMeasurment.angle.get().getRadians()));

        Transform2d scaledTransform = new Transform2d(kalmanTimesDiff.get(0, 0),
                kalmanTimesDiff.get(1, 0),
                is2DPose ? Rotation2d.kZero : Rotation2d.fromRadians(kalmanTimesDiff.get(2, 0)));

        visionUpdates.put(visionMeasurment.timeStamp(), new VisionUpdate(
                new Pose2d(visionMeasurment.pos(), estimateAtTime.getRotation()), estimateAtTime));

        visionUpdates.tailMap(visionMeasurment.timeStamp(), false).entrySet().clear();

        estimatedPose = estimateAtTime.plus(scaledTransform);

    }

    private void updateKalmanFilter() {
        for (int i = 0; i < 3; ++i) {
            stateStdSquared.set(i, 0, stateSTD.get(i, 0) * stateSTD.get(i, 0));
        }

        double[] visionStdSquared = new double[3];
        for (int i = 0; i < 3; ++i) {
            visionStdSquared[i] = visionSTD.get(i, 0) * visionSTD.get(i, 0);
        }

        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        for (int row = 0; row < 3; ++row) {
            if (stateStdSquared.get(row, 0) == 0.0) {
                kalmanMatrix.set(row, row, 0.0);
            } else {
                kalmanMatrix.set(
                        row, row, stateStdSquared.get(row, 0) / (stateStdSquared.get(row, 0)
                                + Math.sqrt(stateStdSquared.get(row, 0) * visionStdSquared[row])));
            }
        }
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public void resetPose() {
        resetPose(new Pose2d());
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPose(pose);
        odometryBuffer.clear();
        visionUpdates.clear();
        estimatedPose = odometry.getPose2d();

    }

    public record OdometryObservation(double timeStamp, Rotation2d gyroAngle, SwerveModulePosition[] swerveModules) {
    }

    public record VisionMeasurment(double timeStamp, Translation2d pos, Optional<Rotation2d> angle) {
    }

    private static final class VisionUpdate {
        // The vision-compensated pose estimate.
        private final Pose2d visionPose;

        // The pose estimated based solely on odometry.
        private final Pose2d odometryPose;

        /**
         * Constructs a vision update record with the specified parameters.
         *
         * @param visionPose   The vision-compensated pose estimate.
         * @param odometryPose The pose estimate based solely on odometry.
         */
        private VisionUpdate(Pose2d visionPose, Pose2d odometryPose) {
            this.visionPose = visionPose;
            this.odometryPose = odometryPose;
        }

        /**
         * Returns the vision-compensated version of the pose. Specifically, changes the
         * pose from being
         * relative to this record's odometry pose to being relative to this record's
         * vision pose.
         *
         * @param pose The pose to compensate.
         * @return The compensated pose.
         */
        public Pose2d compensate(Pose2d pose) {
            var delta = pose.minus(this.odometryPose);
            return this.visionPose.plus(delta);
        }
    }
}
