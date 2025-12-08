package frc.instructions;

import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;

public class step1 {
    //change the setting and sized to ur liking
    public static final int id=5; //this is the robot id
    public static final double maxRobotPower = 0.5; //this is the max power the robot can get in the random function
    public static final double minRobotPower = 0.1;//this is the min power the robot can get in the random function
    public static final double gear_ratio = 1; //this is the robots gear ratio
    public static final Canbus canbus = Canbus.CANIvore;// this is the robots canbus
    public static final double TIME = 0.7;//this is the time between velocity changes
    public static final double JUMP = 0.1;// this is the jump difference

    //change this peremeters, deploy and enable
}
