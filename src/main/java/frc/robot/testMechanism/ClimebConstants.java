package frc.robot.testMechanism;

import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Mechanisms.Arm.ArmState;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;

/** Add your docs here. */
public class ClimebConstants {

    /** the name of the subsystem */
    public static final String NAME = "Climb";
    public static final Canbus CANBUS = Canbus.Rio;

    public static final TalonConfig MOTOR_CONFIG = 
            new TalonConfig(MotorConstants.MOTOR_ID, CANBUS, NAME + "Motor")
            .withInvert(MotorConstants.INVERT)
            .withBrake(MotorConstants.START_NEUTRAL_MODE)
            .withRadiansMotor(ClimbConstants.CLIMB_RATIO);
    /** All the motor constants */
    public static class MotorConstants {
        public static final int MOTOR_ID = 40;
        public static final boolean INVERT = false;
        public static final boolean START_NEUTRAL_MODE = false;
    }
    public static class ClimbConstants {
        public static final double STALL_CURRENT = 35;
        public static final double CLIMB_RATIO = 600;
    }
    public static final int LIMIT_SWITCH_CHANNEL = 4;

    public static enum CLIMB_STATES implements ArmState{
        UP(0.3),
        DOWN(-0.3);

        public final double[] angles;

        CLIMB_STATES(double... angles) {
            this.angles = angles;
        }

        public double[] getValues(){
            return angles;
        }
    }
    
}