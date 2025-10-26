package frc.robot.vision;

import java.text.DateFormat.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Quest extends SubsystemBase {
  private Field2d qField2d;
  private Pose2d questPose;
  private QuestNav questNav;
  
  public Quest() {
    qField2d = new Field2d();
    questNav = new QuestNav();
    questPose = new Pose2d();
    SmartDashboard.putData("quest-map", qField2d);
  } 

  public Pose2d getPose() {
    return questPose;
  }

  @Override
  public void periodic() {
    qField2d.setRobotPose(questPose);
    questNav.commandPeriodic();
    
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
    if (poseFrames.length > 0) {
      questPose = poseFrames[poseFrames.length - 1].questPose();
    }
  }
}