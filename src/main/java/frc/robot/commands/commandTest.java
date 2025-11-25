// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subSystem.test;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class commandTest extends Command {
  /** Creates a new commandTest. */

  test test;
  long start;
  public commandTest(test test) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.test = test;
    addRequirements(test);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = System.currentTimeMillis();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(test.getang() != 90){
      test.setPow(0.2);
    }else{
      test.stop();
    }
    //double kp = 0.05;
    //test.setPow(kp* 90 - test.getang());
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    long stop = System.currentTimeMillis() - start;
    System.out.println(stop);
    test.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return test.getang() > 90;
  }
}
