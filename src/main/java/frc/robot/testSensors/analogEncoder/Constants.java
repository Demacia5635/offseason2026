package frc.robot.testSensors.analogEncoder;

import frc.demacia.utils.Sensors.AnalogEncoderConfig;

public class Constants {
    public static final double OFFSET = 0;
    public static final double FULL_RAGE = 1;
    public static final double MAX_RANGE = 1;
    public static final double MIN_RANGE = 0;
    public static final int CHANNEL = 0;
    public static final String ANALOG_ENCODER_NAME = "test analog encoder";
    public static final AnalogEncoderConfig ANALOG_ENCODER_CONFIG= new AnalogEncoderConfig(CHANNEL,  ANALOG_ENCODER_NAME)
    .withFullRange(FULL_RAGE)
    .withInvert(false)
    .withMaxRange(MAX_RANGE)
    .withMinRange(MIN_RANGE)
    .withOffset(OFFSET);
}
