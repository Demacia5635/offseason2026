// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.chassis.subsystems.Chassis;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestSteer extends Command {
  Chassis chassis;
  public TestSteer(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
    SmartDashboard.putData("TestSteer",this);
    
  }
  double angle = 0;

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("TestSteer/angle", ()-> angle, (x)->this.angle = x);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.setSteerPositions(Math.toRadians(angle));

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
