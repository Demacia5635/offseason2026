// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testChassis.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.Controller.CommandController;
import frc.demacia.utils.chassis.Chassis;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SysIDTesting extends Command {
  CommandController controller;
  Chassis chassis;
  public SysIDTesting(Chassis chassis, CommandController controller) {
    this.chassis = chassis;
    this.controller = controller;
    addRequirements(chassis );
  }
  
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.setDrivePowers(controller.getLeftX());
    chassis.setSteerPowers(controller.getRightY());
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
