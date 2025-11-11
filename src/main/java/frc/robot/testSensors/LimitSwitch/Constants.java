package frc.robot.testSensors.LimitSwitch;

import frc.demacia.utils.Sensors.LimitSwitchConfig;

public class Constants {
    public static final double CHANNEL = 0;
    public static final String LIMIT_SWITCH_NAME = "test limit switch";
    public static final boolean INVERTED = false;
    public static final LimitSwitchConfig LIMIT_SWITCH_CONFIG = new LimitSwitchConfig(0, "Limit Switch")
    .withInvert(false);
}
