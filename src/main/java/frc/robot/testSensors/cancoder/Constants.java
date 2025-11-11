package frc.robot.testSensors.cancoder;

import frc.demacia.utils.Sensors.CancoderConfig;
import com.ctre.phoenix6.CANBus;

public class Constants {
    public static final double OFFSET = 0
    ;
    public static final boolean INVERTED = false;
    public static final int ID=6;
    public static final CANBus CANBUS= new CANBus("canivore");
    public static final String CANCODER_NAME="test cancoder";
    public static final CancoderConfig CANCODER_CONFIG= new CancoderConfig(ID, CANBUS, CANCODER_NAME)
    .withInvert(INVERTED)
    .withOffset(OFFSET);
}
