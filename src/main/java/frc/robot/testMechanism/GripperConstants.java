package frc.robot.testMechanism;

import edu.wpi.first.math.Pair;
import frc.demacia.utils.Mechanisms.Intake.IntakeState;
import frc.demacia.utils.Motors.TalonSRXConfig;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.Sensors.OpticalSensorConfig;
import frc.demacia.utils.Sensors.UltraSonicSensorConfig;

/**
 * All gripper constants divided to diffrent static class based on all the
 * features of the gripper
 */
public class GripperConstants {

    /** the name of the subsystem */
    public static final String NAME = "Gripper";

    /** All the motor constants */
    public static class MotorConstants {
        public static final int MOTOR_ID = 30;
        public static final Canbus CANBUS = Canbus.CANIvore;
        public static final boolean INVERT = false;
        public static final boolean START_NEUTRAL_MODE = true;
        public static final TalonSRXConfig CONFIG = new TalonSRXConfig(MOTOR_ID, NAME)
        .withInvert(INVERT)
        .withBrake(START_NEUTRAL_MODE);
    }

    /** All the sensor constants */
    public static class SensorConstants {
        // public static final int UP_FRONT_SENSOR_CHANNEL = 1;
        // public static final int UP_BACK_SENSOR_CHANNEL = 2;
        public static final int DOWN_SENSOR_CHANNEL = 0;
        public static final OpticalSensorConfig DOWN_CONFOG = new OpticalSensorConfig("down sensor", DOWN_SENSOR_CHANNEL);
        public static final Pair<Integer, Integer> UP_SENSOR_CHANNELS = new Pair<Integer, Integer>(2,3);
        public static final UltraSonicSensorConfig UP_CONFIG = new UltraSonicSensorConfig(UP_SENSOR_CHANNELS.getFirst(), UP_SENSOR_CHANNELS.getSecond(), "up sensor");
        public static final double CORAL_IN_UP_SENSOR = 0.065;
        public static final double CORAL_IN_DOWN_SENSOR = 4.2;
    }

    /** All the constants for the grab command */
    public static class GrabConstants {
        public static final double FEED_POWER = 0.6;
    }
    
    public static class AlignCoralConstants {
        public static final double DOWN_POWER = 0.225;
        public static final double UP_POWER = -0.24;
    }

    /** All the constants for the drop command */
    public static class DropConstants {
        public static final double DROP_POWER = 0.80;
    }

    public static enum GRIPPER_STATES implements IntakeState{
        GRAB(0.6), 
        DROP(0.8), 
        ALIGN_DOWN(0.255), 
        ALIGN_UP(-0.8),
        STOPED(0);

        double power;

        GRIPPER_STATES(double power){
            this.power = power;
        }

        public double[] getValues(){
            return new double[] {power};
        }
    }
}
