package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Quest extends SubsystemBase {
  private Pose2d questPose;
  QuestNav questNav;
  
  public Quest() {
    // For simulation, connect to localhost
    if (RobotBase.isSimulation()) {
      NetworkTableInstance.getDefault().setServer("localhost");
    }
    
    questNav = new QuestNav();

    // Get the latest pose data frames from the Quest
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    if (poseFrames.length > 0) {
      questPose = poseFrames[poseFrames.length - 1].questPose();
    }
  } 

  public Pose2d getPose() {
    return this.questPose;
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();
  }
}