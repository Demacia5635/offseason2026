package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Quest extends SubsystemBase {
  private final Field2d field = new Field2d();
  private final DoubleArraySubscriber positionSub;
  private final DoubleArraySubscriber eulerAnglesSub;
  private Pose2d currentPose = new Pose2d();

  public Quest() {
    NetworkTable questTable = NetworkTableInstance.getDefault().getTable("questnav");
    
    positionSub = questTable.getDoubleArrayTopic("position").subscribe(new double[]{0, 0, 0});
    eulerAnglesSub = questTable.getDoubleArrayTopic("eulerAngles").subscribe(new double[]{0, 0, 0});
    
    SmartDashboard.putData("Quest Field", field);
  }

  public Pose2d getPose() {
    return currentPose;
  }

  @Override
  public void periodic() {
    double[] position = positionSub.get();
    double[] eulerAngles = eulerAnglesSub.get();
    
    if (position.length >= 3 && eulerAngles.length >= 3) {
      double x = position[0];
      double y = position[1];
      double rotationDegrees = eulerAngles[2];
      
      currentPose = new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));
      if(currentPose != null){
        field.setRobotPose(currentPose);
      }
      
      SmartDashboard.putNumber("Quest X", x);
      SmartDashboard.putNumber("Quest Y", y);
      SmartDashboard.putNumber("Quest Rotation", rotationDegrees);
    }
  }
}