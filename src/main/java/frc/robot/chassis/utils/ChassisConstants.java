package frc.robot.chassis.utils;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.Sensors.CancoderConfig;

public class ChassisConstants {
    public static final double CYCLE_DT = 0.02;
    public static final double MAX_DRIVE_VELOCITY = 3.6;
    public static final double MAX_ROTATIONAL_VELOCITY = Math.toRadians(360);
    public static final double MIN_DRIVE_VELOCITY_FOR_ROTATION = 0.2;

    public static class AccelConstants {
        public static final double MAX_LINEAR_ACCEL = 10;

        public static final double MAX_OMEGA_VELOCITY = Math.toRadians(540);
        public static final double MAX_RADIAL_ACCEL = 6;
        public static final double MAX_RADIUS = 0.4;
        public static final double MIN_OMEGA_DIFF = Math.toRadians(20);
        public static final double MAX_DELTA_VELOCITY = MAX_LINEAR_ACCEL * CYCLE_DT;
        public static final double MAX_VELOCITY_TO_IGNORE_RADIUS = MAX_RADIUS * MAX_OMEGA_VELOCITY;
        public static final double MIN_VELOCITY = 1.5;
    }

    public static final int GYRO_ID = 14;
    public static final CANBus CAN_BUS = new CANBus("canivore");
    public static final CANBus GYRO_CAN_BUS = new CANBus("rio");
    public static final double STEER_GEAR_RATIO = 151.0 / 7.0;
    public static final double DRIVE_GEAR_RATIO = 8.14;

    public static final double STEER_KP = 3;
    public static final double STEER_KI = 0.9;
    public static final double STEER_KD = 0;
    public static final double STEER_KS = 0.14817640545050964;
    public static final double STEER_KV = 0.3866402641515461;
    public static final double STEER_KA = 0.05;

    public static final double DRIVE_KP = 19;
    public static final double DRIVE_KI = 0;
    public static final double DRIVE_KD = 0;
    public static final double DRIVE_KS = 0.14677232883614777;
    public static final double DRIVE_KV = 2.947;
    public static final double DRIVE_KA = 0.08058;

    public static final double MOTION_MAGIC_VEL = 15 * 2 * Math.PI;
    public static final double MOTION_MAGIC_ACCEL = 8 * 2 * Math.PI;
    public static final double MOTION_MAGIC_JERK = 160 * 2 * Math.PI;

    public static final double RAMP_TIME_STEER = 0.25;

    public static class SwerveModuleConfigs {
        public final TalonConfig STEER_CONFIG;
        public final TalonConfig DRIVE_CONFIG;
        public final CancoderConfig CANCODER_CONFIG;
        public final Translation2d POSITION;
        public final double STEER_OFFSET;
        public final String NAME;

        public SwerveModuleConfigs(TalonConfig steerConfig, TalonConfig driveConfig, CancoderConfig cancoderConfig,
                Translation2d position, double steerOffset, String name) {
            STEER_CONFIG = steerConfig;
            DRIVE_CONFIG = driveConfig;
            CANCODER_CONFIG = cancoderConfig;
            POSITION = position;
            STEER_OFFSET = steerOffset;
            NAME = name;
        }

        public SwerveModuleConfigs(int swerveId, double steerOffset, double wheelDiameter) {
            switch (swerveId) {
                case 0:
                    NAME = "Front Left";
                    break;
                case 1:
                    NAME = "Front Right";
                    break;
                case 2:
                    NAME = "Back Left";
                    break;
                case 3:
                    NAME = "Back Right";
                    break;

                default:
                    NAME = "";
                    break;
            }
            STEER_CONFIG = new TalonConfig(swerveId * 3 + 2, Canbus.CANIvore, NAME + " Steer")
                    .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, STEER_KA, 0)
                    .withMotionParam(MOTION_MAGIC_VEL, MOTION_MAGIC_ACCEL, MOTION_MAGIC_JERK)
                    .withBrake(true)
                    .withRadiansMotor(STEER_GEAR_RATIO)
                    .withRampTime(RAMP_TIME_STEER);
            DRIVE_CONFIG = new TalonConfig(swerveId * 3 + 1, Canbus.CANIvore, NAME + " Drive")
                    .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, 0)
                    .withBrake(true)
                    .withMeterMotor(DRIVE_GEAR_RATIO ,wheelDiameter);
            CANCODER_CONFIG = new CancoderConfig(swerveId * 3 + 3, CAN_BUS, NAME + " Cancoder");
            POSITION = new Translation2d(
                    swerveId == 0 || swerveId == 1 ? 0.34 : -0.34,
                    swerveId == 0 || swerveId == 2 ? 0.29 : -0.29);
            STEER_OFFSET = steerOffset;
        }
    }

    public static final SwerveModuleConfigs FRONT_LEFT = new SwerveModuleConfigs(
            0,
            -0.33747616603392276926213407751865,
            4*0.0254

    );

    public static final SwerveModuleConfigs FRONT_RIGHT = new SwerveModuleConfigs(
            1,
            0.90198266677216553669500954177338,
            4*0.0254);

    public static final SwerveModuleConfigs BACK_LEFT = new SwerveModuleConfigs(
            2,
            -0.40037085095879042989615619805191,
            4*0.0254);

    public static final SwerveModuleConfigs BACK_RIGHT = new SwerveModuleConfigs(
            3,
            -1.5063685446550771394969298011355,
            4*0.0254);
}