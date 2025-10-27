package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Quest extends SubsystemBase {
  private final Field2d qField2d = new Field2d();
  private final QuestNav questNav = new QuestNav();

  public Quest() {
    SmartDashboard.putData("quest-map", qField2d);
  }

  public Pose2d getPose() {
    return qField2d.getRobotPose();
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic(); // update QuestNav

    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
    if (poseFrames.length > 0) {
      Pose2d latestPose = poseFrames[poseFrames.length - 1].questPose();
      
      SmartDashboard.putNumber("questX", latestPose.getX());
      SmartDashboard.putNumber("questY", latestPose.getY());
      qField2d.setRobotPose(latestPose); // <-- update field directly
      SmartDashboard.putData("quest-map", qField2d);
    }
  }
}
