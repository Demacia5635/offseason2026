// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.UtilsVision;
//import frc.robot.vision.Tag;
import frc.robot.vision.utils.Camera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import static edu.wpi.first.units.Units.Newton;

//import java.lang.reflect.Field;
import java.util.function.Supplier;

//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class objectPos extends SubsystemBase {
  private Camera camera;

  // private Translation2d originToRobot;
  private Translation2d originToObject;
  private Translation2d robotToObject;
  private Translation2d cameraToObject;

  private Field2d field;
  public Pose2d pose;

// NetworkTables communication for each camera
private NetworkTable Table;

  private double tx;
  private double ty;

  // private double height;

  // private double dist;
  private double latency;
  private Supplier<Rotation2d> getRobotAngle;
  // private Pose2d pose;
  private Supplier<Pose2d> robotCurrentPose;
  // private Translation3d robotToCamPosition;



  

  /** Creates a new objectPos. */
  // we dont need tag class this class(objectPos) uses object pos , tx & ty not tag pos , tx & ty
  public objectPos(Camera camera,Supplier<Pose2d> robotCurrentPose, 
  Supplier<Rotation2d> getRobotAngle) {// in the camera there is robotToCamPotition we dont need to take it separately
      this.robotCurrentPose = robotCurrentPose;
      this.getRobotAngle = getRobotAngle;
      this.camera = camera;
      latency = 0;
      // tx = tag.getCameraToTag().getX();
      Table = NetworkTableInstance.getDefault().getTable(camera.getTableName());
      ty = Table.getEntry("ty").getDouble(0.0);
      tx = (-Table.getEntry("tx").getDouble(0.0));
      SmartDashboard.putData("field-tag" + camera.getName(), field);
      
      
  }
  
  private double objectDistanceToCamra(){// this height variable is redundant(dont need to be used we can take from cam)
    double height = camera.getHeight();
    double distX = height*Math.tan(Math.toRadians(ty));
    //dist =distX/Math.cos(Math.toRadians(ty-90));

    return distX/Math.cos(Math.toRadians(tx));//dist;
  }

  private Translation2d objectToRobot(){
    cameraToObject = new Translation2d(objectDistanceToCamra(), Rotation2d.fromRadians(tx));
    //robotToObject = cameraToObject.plus(camera.getRobotToCamPosition().toTranslation2d());
    return cameraToObject.plus(camera.getRobotToCamPosition().toTranslation2d());

  }

  private Translation2d originToObject(){
    robotToObject = robotToObject.rotateBy(getRobotAngle.get());
    originToObject = objectToRobot().plus(robotCurrentPose.get().getTranslation());
    return originToObject;

  }
  public double getTimestamp() {
    return latency;
  }

  @Override
  public void periodic() {
    if (Table.getEntry("tv").getDouble(0.0) != 0) {
      originToObject = originToObject();
      pose = new Pose2d(originToObject(),new Rotation2d());
      field.setRobotPose(pose);
      latency = Table.getEntry("tl").getDouble(0.0) + Table.getEntry("cl").getDouble(0.0);

    }
  }
  public Translation2d getOriginToObject(){
    assert originToObject != null : "vector is null";
    return originToObject;
  }
}
