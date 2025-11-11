// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testMotors.sparkFlex.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.Motors.SparkFlexMotor;
import frc.robot.testMotors.sparkFlex.subsystems.Motor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Set extends Command {
  Motor motor;

  SparkFlexMotor testMotor;

  /** Creates a new Set. */
  public Set(Motor motor) {
    this.motor = motor;
    testMotor = motor.getMotor();
    addRequirements(motor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    testMotor.showConfigMotorCommand();
    testMotor.showConfigPIDFSlotCommand(0);
    testMotor.showConfigMotionVelocitiesCommand();
    testMotor.showControlCommand();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
