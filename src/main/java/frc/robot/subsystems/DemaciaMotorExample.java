package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.Demacia.utils.Motors.MotorInterface;
import frc.Demacia.utils.Motors.TalonConfig;
import frc.Demacia.utils.Motors.TalonMotor;

/*
 * This class is an example of demacia lib
 * 
 * The configuration shoulc be in a Constants file
 * Change the motor id/canbus to match a real motor on the Robot
 * Change the motor configuration to match the motor on the robot
 * 
 * Deploy to Robot
 * In Elastic:
 *  open a new TAB
 *  set the Elastic to shoe submit buttons by default
 *  add the Motor Information - change position/velocity to graph, set label to left
 *  check direction and degrees are correct - if needed update the configuration
 *  add the slow, random and motion command
 *  for the motion command add the Motion Command: field
 *      change the position value to slider, make it longer, set the min/max values and allow updating while draggin
 *      lock the display - for draging the slider
 *  add the PID and MotionVelocities display - change the update buttom type to toggle button, set label to left
 *  
 *  run the Slow power command - mark the voltage when the position starts to move - this should be KS for position (radians/degrees) motor
 *  run the Random power command - let it run for 20-30 seconds
 *  use the log data manage on the PC to transfer the log to the project logs dir
 *  run the Sysid - from Vscode select the Sysid.java file, locate the main function and select RUN
 *  select the downloaded file, select the motor and run the calculate
 *  note the gains - also check that the avg error is low, and the max error is reasnable
 *      records with high error are listed at the message panel
 *      also note the maximum velocity reached
 * 
 *   enter the values in the PID form and update. Note the max velocity and set the Motion Velocities as needed
 * 
 *   run the Motion command
 *      use the slider to move the motor angle
 *      look at the graph for normal vehaviour
 *      look at the ClosedLoopError value - it should be small
 *      change parameters to get a good profile
 *      update the configuration of the motor
 * 
 *  re-deploy and check the motion
 * 
 * Note on position/radians/degrees motors:
 *     the main issue is the accuracy - for accuracy we need ggod paramters for very slow velocities.
 *     it is best to set the kS to the minimum power that generate movement - Static Friction vs Dynamic friction for Velocity motor
 *     The kA can be larger - it will increase acceleration
 *     The kP can be larger for faster correction
 *   
 */

public class DemaciaMotorExample extends SubsystemBase {

  public static class Example { // Should be in Constants file
    public static final TalonConfig MOTOR_CONFIG = new TalonConfig(11, Canbus.Rio, "TalonExample")
      .withBrake(true)
      .withCurrent(20) // Maximum current in Apmpers
      .withInvert(true)
      .withDegreesMotor(12.8) // the motor gear ratio - all data in degrees
      .withMotionParam(1500, 4000, 5000) // set the motion parametrs
      .withPID(0.01, 0, 0, 0.13, 0.0045,0.0005, 0) // the PID/FF gains
      .withRampTime(0.2) // time from 0 to full power
      .withVolts(7);
}

    // Define the motor 
    MotorInterface motor;

    // Constructor
    public DemaciaMotorExample() {
        super();
        motor = new TalonMotor(Example.MOTOR_CONFIG);
        motor.showSysidCommands(this);
    }
}   