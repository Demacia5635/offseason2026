// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.UtilsVision;
import frc.robot.vision.utils.Camera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class objectPos extends SubsystemBase {
  private Camera camera;
  private Translation2d robotToObject;
  private Translation2d cameraToObject;
  private Supplier<Rotation2d> getRobotAngle;
  private double camToObjectYaw;
  private double ty;
  private double id;
  private double height;
  private double distX;
  private Translation2d originToRobot;
  private Translation2d origintoObject;
  public Pose2d pose;
  private Supplier<Pose2d> robotCurrentPose;
  private Translation3d robotToCamPosition;
  private double dist;
  private double tx;
  

  /** Creates a new objectPos. */
  public objectPos(Camera camera,Supplier<Pose2d> robotCurrentPose,Translation3d robotToCamPosition, Supplier<Rotation2d> getRobotAngle) {
      this.robotCurrentPose = robotCurrentPose;
      this.getRobotAngle = getRobotAngle;
  }
  
  private double objectDistanceToCamra(){
    height = camera.getRobotToCamPosition().getZ();
    distX = height/Math.tan(Math.toRadians(ty-90));
    dist =distX/Math.cos(Math.toRadians(ty-90));

    return dist;
  }

  private Translation2d objectToRobot(){
    cameraToObject = new Translation2d(objectDistanceToCamra(),new Rotation2d(tx));
    robotToObject = cameraToObject.plus(camera.getRobotToCamPosition().toTranslation2d());
    return robotToObject;

  }

  private Translation2d objectToOrigin(){
    robotToObject = objectToRobot().rotateBy(getRobotAngle.get());
    origintoObject = robotToObject.plus(robotCurrentPose.get().getTranslation());

    return originToRobot;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
