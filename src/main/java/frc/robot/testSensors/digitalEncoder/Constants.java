package frc.robot.testSensors.digitalEncoder;

import frc.demacia.utils.Sensors.DigitalEncoderConfig;

public class Constants {
    public static final double OFFSET = 0;
    public static final double SCALAR = 1;
    public static final boolean INVERTED = false;
    public static final double MIN_RANGE = 0;
    public static final double MAX_RANGE = 1;
    public static final double FREQUENCY = 1000;
    public static final int CHANNAL=0;
    public static final String DIGITAL_ENCODER_NAME="digital encoder";
    public static final DigitalEncoderConfig DIGITAL_ENCODER_CONFIG= new DigitalEncoderConfig(CHANNAL, DIGITAL_ENCODER_NAME)
    .withFrequency(FREQUENCY)
    .withInvert(INVERTED).
    withMaxRange(MAX_RANGE)
    .withMinRange(MIN_RANGE)
    .withScalar(SCALAR)
    .withOffset(OFFSET);
}
