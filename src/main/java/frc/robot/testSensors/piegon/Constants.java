package frc.robot.testSensors.piegon;

import frc.demacia.utils.Sensors.PigeonConfig;
import com.ctre.phoenix6.CANBus;

public class Constants {
    public static final boolean INVERTED= false;
    public static final double PICH_OFFSET = 0;
    public static final double ROLL_OFFSET = 0;
    public static final double YAW_OFFSET = 171;
    public static final double XSCALAR = 1;
    public static final double YSCALAR = 1;
    public static final double ZSCALAR = 1;
    public static final boolean COMPASS = false;
    public static final boolean temperatureCompensation = false;
    public static final boolean noMotionCalibration = false;
    public static final int ID=14;
    public static final CANBus CANBUS= new CANBus("rio");
    public static final String PIZZAPIZZAPIZZA_PIGEON_NAME="PIZZA PIGEON";
    public static final PigeonConfig PIGEON_CONFIG= new PigeonConfig(ID, CANBUS, PIZZAPIZZAPIZZA_PIGEON_NAME)
    .withPitchOffset(PICH_OFFSET)
    .withRollOffset(ROLL_OFFSET)
    .withYawOffset(YAW_OFFSET)
    .withInvert(INVERTED);
    
}            