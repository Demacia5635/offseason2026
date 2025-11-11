package frc.robot.testMechanism;

import frc.demacia.utils.Mechanisms.Arm.ArmState;
import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;

public class ArmConstants {
    
    public static final String NAME = "Arm";
    public static class ArmAngleMotorConstants {
        /* all the main configs of the motor */
        public static final int ID = 20;
        public static final Canbus CAN_BUS = Canbus.Rio;
        public static final String NAME = "Arm Angle Motor";

        /* the pid and ff constants of the motor */
        public static final double KP = 22;
        public static final double KI = 1.0;
        public static final double KD = 0.75;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KG = 0;

        /* the motion magic constants of the motor */
        public static final double MOTION_MAGIC_VELOCITY = 1.5;
        public static final double MOTION_MAGIC_ACCELERATION = 3;
        public static final double MOTION_MAGIC_JERK = 6;

        /* the channel of the limit switch of the arm angle motor */
        public static final int LIMIT_SWITCH_CHANNEL = 0;

        /* the basic configues of the motor */
        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERTED = false;
        public static final double GEAR_RATIO = 36.0 * (105.0 / 16.0);

        /* the ramp time of the motor */
        public static final double RAMP_TIME = 0.5;

        /*
         * all the angles of the motor
         * base -> where the limit switch
         * back limit -> the minimum angle
         * forward limit -> the maximum angle
         */
        public static final double BASE_ANGLE = Math.toRadians(33.7);
        public static final double BACK_LIMIT = Math.toRadians(33.7);
        public static final double FWD_LIMIT = 2.904541015625;

        /* The config of the motors based on the constants above */
        public static final TalonConfig CONFIG = new TalonConfig(ID, CAN_BUS, NAME)
                .withPID(KP, KI, KD, KS, KV, KA, KG)
                .withMotionParam(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
                .withBrake(IS_BRAKE)
                .withInvert(IS_INVERTED)
                .withRadiansMotor(GEAR_RATIO)
                .withRampTime(RAMP_TIME);
    }

    public static class GripperAngleMotorConstants {
        /* All the main configs of the motor */
        public static final int ID = 21;
        public static final Canbus CAN_BUS = Canbus.Rio;
        public static final String NAME = "Gripper Angle Motor";

        /* the pid and ff of the motor */
        public static final double KP = 9.5;
        public static final double KI = 1.9;
        public static final double KD = 0.25;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KG = 0;

        /* the motion magic constants of the motor */
        public static final double MOTION_MAGIC_VELOCITY = 0;
        public static final double MOTION_MAGIC_ACCELERATION = 0;
        public static final double MOTION_MAGIC_JERK = 0;

        /* the channel of the absolute sensor */
        public static final int ABSOLUTE_SENSOR_CHANNEL = 1;

        /* all the basic configs of the motor */
        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERTED = false;
        public static final double GEAR_RATIO = 36.0 * (47.0 / 20.0);

        /* the ramp time of the motor */
        public static final double RAMP_TIME = 0.5;

        /*
         * all the angles of the motor
         * base -> where the limit switch
         * back limit -> the minimum angle
         * forward limit -> the maximum angle
         */
        public static final double ENCODER_BASE_ANGLE = 3.1763315220836814 - 1.5*Math.PI;
        public static final double BACK_LIMIT = 3.7;
        public static final double FWD_LIMIT = 5.4;

        /* The config of the motor based on the constants above */
        public static final TalonConfig CONFIG = new TalonConfig(ID, CAN_BUS, NAME)
                .withPID(KP, KI, KD, KS, KV, KA, KG)
                .withPID(1, KP*3, KI, KD, KS, KV, KA, KG)
                .withMotionParam(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
                .withBrake(IS_BRAKE)
                .withInvert(IS_INVERTED)
                .withRadiansMotor(GEAR_RATIO)
                .withRampTime(RAMP_TIME);
    }

    public static enum ARM_STATES implements ArmState{
        L1(Math.toRadians(37.3), 4.6),
        L2(1.8, 4.4),
        L3(2.64208984375, 4.729228859906724),
        PRE_ALGAE_BOTTOM(2.5, 2.5),
        PRE_ALGAE_TOP(1.8, 3.7),
        AFTER_ALGAE_BOTTOM(1.6, 2.5),
        AFTER_ALGAE_TOP(2.5, 4.6),
        CORAL_STATION(1.6, 5.3),
        CLIMB(2.766 ,5.4),
        STARTING(Math.toRadians(33.7), 3.64),
        IDLE(0,0);

        public final double[] angles;

        ARM_STATES(double... angles) {
            this.angles = angles;
        }

        public double[] getValues(){
            return angles;
        }
    }
}
