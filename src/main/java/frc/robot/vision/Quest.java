// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Quest extends SubsystemBase {
  private double PoseFrame;
  private Pose2d questPose;
  QuestNav questNav = new QuestNav();
  public Quest() {

    // Get the latest pose data frames from the Quest
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    if (poseFrames.length > 0) {
    // Get the most recent Quest pose
      questPose = poseFrames[poseFrames.length - 1].questPose();

    }
  } 

  public Pose2d getPose() {
    return this.questPose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    questNav.commandPeriodic();
  }

}
