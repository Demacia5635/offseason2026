// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.motorConstants;
import frc.robot.subsystem.MotorForSysid;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MotorCommand extends Command {
  MotorForSysid motor;
  private double power = 0;
  private double time ; 
  private Timer timer;

  /** Creates a new motorcommand. */
  public MotorCommand(MotorForSysid motor) {
    this.motor = motor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motor);
    time = 0.5;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   power = Math.random() *(motorConstants.MAX_POWER-motorConstants.MIN_POWER)+(motorConstants.MIN_POWER) ;
   if (timer.get()>=time){
    power = Math.random() *(motorConstants.MAX_POWER-motorConstants.MIN_POWER)+(motorConstants.MIN_POWER);
    timer.reset();
   }
    motor.setPower(power);
   

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
