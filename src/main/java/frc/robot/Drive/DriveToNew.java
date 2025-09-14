// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToNew extends Command {
  DriveSubsystem drive;
  Pose2d pose;
  double kP = 1.2;
  Translation2d toEndVector;
  boolean usePoseAngle;
  double omega;


  public DriveToNew(DriveSubsystem drive, Pose2d pose, boolean usePoseAngle) {
    this.drive = drive;
    this.pose = pose;
    toEndVector = new Translation2d(); 
    this.usePoseAngle = usePoseAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    toEndVector = pose.minus(drive.pose).getTranslation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    toEndVector.set(pose.getX() - drive.pose.getX(), pose.getY() - drive.pose.getY());
    
    if(usePoseAngle) omega = toEndVector.getAngle().getRadians() * kP;
    else omega = Rotation2d.kPi.getRadians();

    double v = Math.min(toEndVector.getNorm() * kP, 2);
    
    drive.setSpeeds(new ChassisSpeeds(v * toEndVector.getAngle().getCos(), v * toEndVector.getAngle().getSin(), omega));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setSpeeds(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return toEndVector.getNorm() <= 0.05;
  }
}
