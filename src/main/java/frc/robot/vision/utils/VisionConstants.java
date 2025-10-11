// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Constants and configuration values for AprilTag vision processing.
 * This class contains the physical layout and properties of AprilTags on the
 * field,
 * as well as camera mounting parameters.
 */
public class VisionConstants {

        // Heights of different AprilTag groups from the ground
        private static double BARGE_TAG_HIGHT = inchToMeter(73.54);
        private static double REEF_TAG_HIGHT = inchToMeter(12.13);
        private static double STATION_TAG_HIGHT = inchToMeter(58.50);
        private static double SIDE_TAG_HIGHT = inchToMeter(51.25);

        public static final double BEST_RELIABLE_DISTANCE = 1; // meters - high confidence range
        public static final double WORST_RELIABLE_DISTANCE = 4; // meters - Low confidence range

        public static final double BEST_RELIABLE_SPEED = 1; // meter a second - high confidence range
        public static final double WORST_RELIABLE_SPEED = 3; // meter a second - Low confidence range

        /**
         * Array of AprilTag positions relative to field origin (0,0).
         * Each Translation2d represents X,Y coordinates in meters.
         * Index corresponds to AprilTag ID (0 is unused).
         * Coordinates are converted from inches to meters using inchToMeter().
         */
        public static Translation2d[] O_TO_TAG = { null, // 0
                        new Translation2d(inchToMeter(657.37), inchToMeter(25.80)), // 1
                        new Translation2d(inchToMeter(657.37), inchToMeter(291.20)), // 2
                        new Translation2d(inchToMeter(455.15), inchToMeter(317.15)), // 3
                        new Translation2d(inchToMeter(365.20), inchToMeter(241.64)), // 4
                        new Translation2d(inchToMeter(365.20), inchToMeter(75.39)), // 5
                        new Translation2d(inchToMeter(530.49), inchToMeter(130.17)), // 6
                        new Translation2d(inchToMeter(546.87), inchToMeter(158.50)), // 7
                        new Translation2d(inchToMeter(530.49), inchToMeter(186.83)), // 8
                        new Translation2d(inchToMeter(497.77), inchToMeter(186.83)), // 9
                        new Translation2d(inchToMeter(481.39), inchToMeter(158.50)), // 10
                        new Translation2d(inchToMeter(497.77), inchToMeter(130.17)), // 11
                        new Translation2d(inchToMeter(33.51), inchToMeter(25.80)), // 12
                        new Translation2d(inchToMeter(33.51), inchToMeter(291.20)), // 13
                        new Translation2d(inchToMeter(325.68), inchToMeter(241.64)), // 14
                        new Translation2d(inchToMeter(325.68), inchToMeter(75.39)), // 15
                        new Translation2d(inchToMeter(235.73), inchToMeter(-0.15)), // 16
                        new Translation2d(inchToMeter(160.39), inchToMeter(130.17)), // 17
                        new Translation2d(inchToMeter(144.00), inchToMeter(158.50)), // 18
                        new Translation2d(inchToMeter(160.39), inchToMeter(186.83)), // 19
                        new Translation2d(inchToMeter(193.10), inchToMeter(186.83)), // 20
                        new Translation2d(inchToMeter(209.49), inchToMeter(158.50)), // 21
                        new Translation2d(inchToMeter(193.10), inchToMeter(130.17)),// 22

        };
        public static Rotation2d[] TAG_ANGLE = { null, // 0
                        Rotation2d.fromDegrees(126), // 1
                        Rotation2d.fromDegrees(234), // 2
                        Rotation2d.fromDegrees(270), // 3
                        Rotation2d.fromDegrees(0), // 4
                        Rotation2d.fromDegrees(0), // 5
                        Rotation2d.fromDegrees(300), // 6
                        Rotation2d.fromDegrees(0), // 7
                        Rotation2d.fromDegrees(60), // 8
                        Rotation2d.fromDegrees(120), // 9
                        Rotation2d.fromDegrees(180), // 10
                        Rotation2d.fromDegrees(240), // 11
                        Rotation2d.fromDegrees(54), // 12
                        Rotation2d.fromDegrees(306), // 13
                        Rotation2d.fromDegrees(180), // 14
                        Rotation2d.fromDegrees(180), // 15
                        Rotation2d.fromDegrees(90), // 16
                        Rotation2d.fromDegrees(240), // 17
                        Rotation2d.fromDegrees(180), // 18
                        Rotation2d.fromDegrees(120), // 19
                        Rotation2d.fromDegrees(60), // 20
                        Rotation2d.fromDegrees(0), // 21
                        Rotation2d.fromDegrees(300),// 22
        };

        /**
         * Array of AprilTag heights from the ground.
         * Each value corresponds to either LOW, MID, or HIGH mounting position.
         * Index corresponds to AprilTag ID (0 is unused).
         */
        public static double[] TAG_HEIGHT = { 0, // 0
                        STATION_TAG_HIGHT, // 1
                        STATION_TAG_HIGHT, // 2
                        SIDE_TAG_HIGHT, // 3
                        BARGE_TAG_HIGHT, // 4
                        BARGE_TAG_HIGHT, // 5
                        REEF_TAG_HIGHT, // 6
                        REEF_TAG_HIGHT, // 7
                        REEF_TAG_HIGHT, // 8
                        REEF_TAG_HIGHT, // 9
                        REEF_TAG_HIGHT, // 10
                        REEF_TAG_HIGHT, // 11
                        STATION_TAG_HIGHT, // 12
                        STATION_TAG_HIGHT, // 13
                        BARGE_TAG_HIGHT, // 14
                        BARGE_TAG_HIGHT, // 15
                        SIDE_TAG_HIGHT, // 16
                        REEF_TAG_HIGHT, // 17
                        REEF_TAG_HIGHT, // 18
                        REEF_TAG_HIGHT, // 19
                        REEF_TAG_HIGHT, // 20
                        REEF_TAG_HIGHT, // 21
                        REEF_TAG_HIGHT,// 22
        };

        public static final Translation2d REEF_TAG_TO_RIGHT_SCORING = new Translation2d(-0.55, -0.160);
        public static final Translation2d REEF_TAG_TO_LEFT_SCORING = new Translation2d(-0.55, 0.160);
        public static final Translation2d INTAKE_TAG_TO_LEFT_SCORING = new Translation2d(-0.85, 0.0);

        /**
         * Converts a measurement from inches to meters
         * 
         * @param inch Value in inches
         * @return Value in meters
         */
        public static double inchToMeter(double inch) {
                return inch * 0.0254;
        }

        // 0 is LEFT TAG(ll2), 1 is RIGHT TAG, 2 is SIDE TAG, 3 is NOTE.

        // note for faniel and tomer in 14/2/25 -
        // the vision code suled work for the maunt noga printed if somthing
        // dose not work reaf the next coments

        // if the dist calc do not work, chaneg to 90-the number(only for id 1 and 2) in
        // line 141

        // if the robot tihnk it is behined the tag swap id 1 with id 2 in line 150
        // TAG Camera mounting configuration

        public static final double MIN_CROP = 0.5;
        public static final double MAX_CROP = 0.6;
        public static final double CROP_CONSTAT = 0.35;
        public static final double PREDICT_X = -0.1;
        public static final double PREDICT_Y = 0.15;
        public static final double PREDICT_OMEGA = 0.2;

}