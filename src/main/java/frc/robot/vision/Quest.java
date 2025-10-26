package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Quest extends SubsystemBase {
  private Pose2d questPose;
  private QuestNav questNav;
  
  public Quest() {
    questNav = new QuestNav();
    questPose = new Pose2d();
  } 

  public Pose2d getPose() {
    return questPose;
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();
    
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
    if (poseFrames.length > 0) {
      questPose = poseFrames[poseFrames.length - 1].questPose();
    }
  }
}