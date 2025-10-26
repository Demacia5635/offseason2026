// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.UtilsVision;
import frc.robot.vision.Tag;
import frc.robot.vision.utils.Camera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class objectPos extends SubsystemBase {
  private Camera camera;
  private Translation2d robotToObject;
  private Translation2d cameraToObject;
  private Supplier<Rotation2d> getRobotAngle;
  private double ty;
  private double height;
  private double distX;
  private Translation2d originToRobot;
  private Translation2d originToObject;
  private Pose2d pose;
  private Supplier<Pose2d> robotCurrentPose;
  private Translation3d robotToCamPosition;
  private double dist;
  private double tx;
  

  /** Creates a new objectPos. */
  public objectPos(Camera camera,Supplier<Pose2d> robotCurrentPose,Translation3d robotToCamPosition, 
  Supplier<Rotation2d> getRobotAngle, Tag tag) {
      this.robotCurrentPose = robotCurrentPose;
      this.getRobotAngle = getRobotAngle;
      this.camera = camera;
      tx = tag.getCameraToTag().getX();


      
  }
  
  private double objectDistanceToCamra(){
    height = camera.getRobotToCamPosition().getZ();
    distX = height/Math.tan(Math.toRadians(ty-90));
    //dist =distX/Math.cos(Math.toRadians(ty-90));

    return distX/Math.cos(Math.toRadians(ty-90));//dist;
  }

  private Translation2d objectToRobot(){
    cameraToObject = new Translation2d(objectDistanceToCamra(), Rotation2d.fromRadians(tx));
    //robotToObject = cameraToObject.plus(camera.getRobotToCamPosition().toTranslation2d());
    return cameraToObject.plus(camera.getRobotToCamPosition().toTranslation2d());

  }

  private Translation2d originToObject(){
    //robotToObject = objectToRobot().rotateBy(getRobotAngle.get());
    // = robotToObject.plus(robotCurrentPose.get().getTranslation());
    return robotCurrentPose.get().getTranslation().minus(objectToRobot());
    // return objectToRobot().rotateBy(getRobotAngle.get()).plus(robotCurrentPose.get().getTranslation());
  }

  @Override
  public void periodic() {
    originToObject = originToObject();

    
  }

  public Translation2d getOriginToObject(){
    assert originToObject == null : "vector is null";
    return originToObject;
  }
}
