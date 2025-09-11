package frc.robot.Drive;

import frc.Demacia.Geometry.Translation2d;
import frc.Demacia.utils.Motors.TalonConfig;
import frc.Demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.Demacia.utils.Sensors.CancoderConfig;

public class Constants {

    public static final double MK4i_STEER_RATIO = 150.0 / 7.0;
    public static final double MK4_STEER_RATIO = 12.8;
    public static final double L1_DRIVE_RATIO = 8.14;
    public static final double L2_DRIVE_RATIO = 6.75;
    public static final double WHEEL_DIAMETER = 4 * 0.0254;

    public static final double X_POSITION = 0.35;
    public static final double Y_POSITION = 0.3;

    public static final int GYRO_ID = 14;
    public static final Canbus GYRO_CANBUS = Canbus.Rio;

    public static double CYCLE_TIME = 0.02;

    public static final double MAX_SPEED = 3.7;
    public static final double MAX_OMEGA = 6; // Radians per second
    public static final double MAX_X_ACCELERATION = 7;
    public static final double MAX_X_VELOCITY_CHANGE = MAX_X_ACCELERATION * CYCLE_TIME;
    public static final double MAX_Y_ACCELERATION = 5;
    public static final double MAX_Y_VELOCITY_CHANGE = MAX_Y_ACCELERATION * CYCLE_TIME;

    public static final double STEER_TO_DISTANCE_RATIO = 0.14/360.0; // 14 cm for 1 steer rotation

    public static double MAX_STEER_CORRECTION = 25.0;
    public static double STATE_STEER_MULTIPLIER = 1.7;

    public static boolean CANCODER_INVERTED = false;

    public static final TalonConfig BASE_STEER_CONFIG = new TalonConfig(0, Canbus.CANIvore, "BASE_STEER")
            .withBrake(true)
            .withCurrent(15)
            .withDegreesMotor(MK4i_STEER_RATIO)
            .withInvert(false)
            .withPID(0.05, 0, 0, 0.2, 0.007, 0.0004, 0)
            .withRampTime(0.2)
            .withMotionParam(1000, 2500, 6000)
            .withVolts(8);
    public static final TalonConfig BASE_DRIVE_CONFIG = new TalonConfig(0, Canbus.CANIvore, "BASE_DRIVE")
            .withBrake(true)
            .withCurrent(30)
            .withMeterMotor(L1_DRIVE_RATIO, WHEEL_DIAMETER)
            .withInvert(false)
            .withPID(2  , 0, 0, 0.1, 2.93, 0.13, 0)
            .withRampTime(0.2)
            .withMotionParam(3.5, 6.5, 10)
            .withVolts(12);

    static class ModuleConfig {
        TalonConfig steerConfig;
        TalonConfig driveConfig;
        CancoderConfig cancoderConfig;
        double cancoderOffset;
        Translation2d positionRelativeToRobotCenter;
        String name;

        ModuleConfig(int steerId, int driveId, int cancoderId, double xPosition, double yPosition,
                double cancoderOffset) {
            positionRelativeToRobotCenter = new Translation2d(xPosition, yPosition);
            name = (xPosition > 0 ? "Front" : "Back") + (yPosition > 0 ? "Left" : "Right");
            cancoderConfig = new CancoderConfig(cancoderId, BASE_DRIVE_CONFIG.canbus, name + "/Cancoder").withInvert(CANCODER_INVERTED);
            this.cancoderOffset = cancoderOffset;
            steerConfig = new TalonConfig(steerId, name + "/Steer", BASE_STEER_CONFIG);
            driveConfig = new TalonConfig(driveId, name + "/Drive", BASE_DRIVE_CONFIG);
        }
    }

    public static final ModuleConfig[] CONFIGS = {
        new ModuleConfig(2, 1, 3, X_POSITION, Y_POSITION, 141.2),
        new ModuleConfig(5, 4, 6, X_POSITION, -Y_POSITION, -178.8),
        new ModuleConfig(8, 7, 9, -X_POSITION, Y_POSITION, -22.9),
        new ModuleConfig(11, 10, 12, -X_POSITION, -Y_POSITION, 139.7)
};
public static final ModuleConfig[] CONFIGS1 = {
    new ModuleConfig(11, 10, 12, X_POSITION, Y_POSITION, 150.6)
};

}
