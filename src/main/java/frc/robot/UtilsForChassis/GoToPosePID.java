// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.UtilsForChassis;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToPosePID extends Command {
  Pose2d targetPose;
  Consumer<ChassisSpeeds> setVelocities;
  Supplier<Pose2d> currentPose;
  Translation2d diffVector;
  Rotation2d diffAngle;
  PIDController drivePID;
  PIDController rotatePID;
  public GoToPosePID(Pose2d targetPose, Consumer<ChassisSpeeds> setVelocities, Supplier<Pose2d> currentPose) {
    this.targetPose = targetPose;
    this.setVelocities = setVelocities;
    this.currentPose = currentPose;
    this.drivePID = new PIDController(-1, -1, -1);
    this.rotatePID = new PIDController(-1, -1, -1);
    


  }

  @Override
  public void initialize() {
    this.diffVector = targetPose.minus(currentPose.get()).getTranslation();

  }

  
  @Override
  public void execute() {
    Pose2d curPose = currentPose.get();
    diffVector = targetPose.minus(curPose).getTranslation();
    diffAngle = targetPose.getRotation().minus(curPose.getRotation());

    setVelocities.accept(new ChassisSpeeds(drivePID.calculate(diffVector.getX()), drivePID.calculate(diffVector.getY()), rotatePID.calculate(diffAngle.getRadians())));

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    setVelocities.accept(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return diffVector.getNorm() < 0.03 && Math.abs(diffAngle.getRadians()) < Math.toDegrees(0.5);
  }
}
