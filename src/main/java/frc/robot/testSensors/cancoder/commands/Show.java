// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testSensors.cancoder.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.testSensors.cancoder.subsystems.Cancoder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Show extends Command {
  Cancoder cancoder;

  frc.demacia.utils.Sensors.Cancoder testCancoder;

  /** Creates a new Set. */
  public Show(Cancoder cancoder) {
    this.cancoder = cancoder;
    testCancoder = cancoder.getCancoder();
    addRequirements(cancoder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    testCancoder.showConfigMotorCommand();
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
