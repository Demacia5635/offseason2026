// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.utils;

import static frc.robot.vision.utils.VisionConstants.TAG_ANGLE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.Tag;


/** Add your docs here. */
public class VisionFuse {

    private Tag[] tags = new Tag[4];

    public VisionFuse(Tag right, Tag feeder, Tag barge, Tag left) {

        this.tags[0] = right;
        this.tags[1] = feeder;
        this.tags[2] = barge;
        this.tags[3] = left;

    }

    private double getColectedConfidence() {
        double confidence = 0;
        for (Tag tag : tags) {
            if (tag.getPose() != null) {
                confidence += tag.getPoseEstemationConfidence();
            }
        }
        return confidence;
    }

    private double normalizeConfidence(double confidence) {
        return getColectedConfidence() == 0 ? 0 : confidence * (1 / getColectedConfidence());
    }

    public Pose2d getPoseEstemation() {
        double x = 0;
        double y = 0;
        double angle = 0;
        for (Tag tag : tags) {
            if (tag.getPose() == null)
                continue;
            x += tag.getPose().getX() * normalizeConfidence(tag.getPoseEstemationConfidence());
            y += tag.getPose().getY() * normalizeConfidence(tag.getPoseEstemationConfidence());
            angle += tag.getPose().getRotation().getRadians() * normalizeConfidence(tag.getPoseEstemationConfidence());
        }
        return (x == 0 && y == 0) ? null : new Pose2d(x, y, (getRotationEstimation() == null) ? new Rotation2d(angle) : getRotationEstimation());
    }

    public Rotation2d getRotationEstimation() {
        return Rotation2d.fromDegrees(tags[2].getAngle());
    }

    public double getVisionTimestamp() {
        double timestamp = 0;
        for (Tag tag : tags) {
            if (tag.getPose() != null) {
                timestamp += tag.getTimestamp() * normalizeConfidence(tag.getPoseEstemationConfidence());
            }
        }
        return timestamp;
    }

    private Integer getBestCamera() {
        Integer bestCamera = null;
        double highestConfidence = 0.0;

        for (int i = 0; i < tags.length; i++) {
            double currentConfidence = tags[i].getPoseEstemationConfidence();

            if (currentConfidence > highestConfidence && (currentConfidence > 0.1 || tags[i].is3D)) {
                highestConfidence = currentConfidence;
                bestCamera = i;
            }
        }

        return bestCamera;
    }

    public Rotation2d getVisionEstimatedAngle() {
        return getBestCamera() != null ? tags[getBestCamera()].getRobotAngle() : null;
    }

    public double getVisionConfidence() {
        return Math.max(getColectedConfidence(), 1);
    }

    public Rotation2d get2dAngle() {
        // Ensure that 'tags' is not empty or has a valid index before accessing
        if (tags != null && tags.length > 3 && tags[0].getCameraToTag() != null
                && tags[3].getCameraToTag() != null
                && tags[0].getTagId() == tags[3].getTagId()) {
            return tags[0].getCameraToTag().minus(tags[3].getCameraToTag())
                    .getAngle().rotateBy(Rotation2d.fromDegrees(90))
                    .plus(TAG_ANGLE[tags[3].getTagId()]);
        } else {
            return null; // If the tags array is not correctly initialized, return null.
        }
    }

    public void set3D(boolean is3D){
        for (Tag tag : tags) {
            tag.set3D(is3D);
        }
    }

    public double get3DAngle() {
        Integer bestCamera = getBestCamera();
        return bestCamera != null ? tags[bestCamera].getAngle() : 0;
    }
    
    
}