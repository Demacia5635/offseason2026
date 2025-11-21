// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.demacia.utils.Controller.CommandController;
import frc.demacia.utils.Controller.CommandController.ControllerType;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.Mechanisms.Arm;
import frc.demacia.utils.Mechanisms.Intake;
import frc.demacia.utils.Motors.MotorInterface;
import frc.demacia.utils.Motors.TalonFXMotor;
import frc.demacia.utils.Motors.TalonSRXMotor;
import frc.demacia.utils.Sensors.OpticalSensor;
import frc.demacia.utils.Sensors.SensorInterface;
import frc.demacia.utils.Sensors.UltraSonicSensor;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.testChassis.ChassisConstants;
import frc.robot.testChassis.commands.DriveCommand;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  Arm arm;
  Arm clibeb;
  Intake gripper;

  Chassis chassis;
  DriveCommand driveCommand;

  
  public static CommandController driverController;

  public static boolean isComp = DriverStation.isFMSAttached();
  private static boolean hasRemovedFromLog = false;

  public static int N_CYCLE = 0;
  public static double CYCLE_TIME = 0.02;

  // The robot's subsystems and commands are defined here...


  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    new LogManager();
    driverController = new CommandController(0, ControllerType.kPS5);

    chassis = new Chassis(ChassisConstants.CHASSIS_CONFIG);
    chassis.setDefaultCommand(new DriveCommand(chassis, driverController));
    // chassis = new Chassis(ChassisConstants.CHASSIS_CONFIG);
    // driveCommand = new DriveCommand(chassis, driverController);

    // setMechanism();

    // Configure the trigger bindings
    // testMotor.setDefaultCommand(new TestMotorCommand(testMotor,5););
    configureBindings();
  }

  public static boolean isComp() {
    return isComp;
  }

  public static void setIsComp(boolean isComp) {
    RobotContainer.isComp = isComp;
    if(!hasRemovedFromLog && isComp) {
      hasRemovedFromLog = true;
      LogManager.removeInComp();
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // arm.setDefaultCommand(arm.toStateCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
