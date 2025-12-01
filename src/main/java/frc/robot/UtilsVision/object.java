// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.UtilsVision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.UtilsVision.Camera;

public class object extends SubsystemBase {
  private Translation2d robotToObject;
  private Translation2d cameraToObject;
  private Translation2d OriginToObject;

  private NetworkTable Table;


  private double camToObjectYaw;
  private double camToObjectPitch;

  
  private Supplier<Rotation2d> getRobotAngle;
  private Supplier<Pose2d> robotCurrentPose;


  private Camera camera;
  private Pose2d objectPose;

  public object(Camera camera, Supplier<Rotation2d> getRobotAngle,Supplier<Pose2d> robotCurrentPose) {
    this.getRobotAngle = getRobotAngle;
    this.robotCurrentPose = robotCurrentPose;

    this.camera = camera;
    Table = NetworkTableInstance.getDefault().getTable(camera.getTableName());
  }

  @Override
  public void periodic() {
    camToObjectPitch = Table.getEntry("ty").getDouble(0.0);
    camToObjectYaw = (-Table.getEntry("tx").getDouble(0.0)) + camera.getYaw();
    if(Table.getEntry("tv").getDouble(0.0) != 0){
      objectPose = new Pose2d(getOriginToObject(), getRobotAngle.get());
    }
    
  }

  public Pose2d getPose2d(){
    return objectPose;
  }
  public double getDistcameraToObject(){
    double alpha = camToObjectPitch + camera.getPitch();
    Math.toRadians(alpha);
    double distX =  Math.abs(camera.getCamHeight()*(Math.tan(alpha)));
    return distX/( Math.cos(Math.toRadians( (camera.getPitch()+camToObjectPitch) ) ) );
  }

  public Translation2d getRobotToObject(){
    cameraToObject = new Translation2d(getDistcameraToObject(),camera.getPitch()+camToObjectPitch);
    robotToObject = new Translation2d(camera.getRobotToCamPosition().getX(), camera.getRobotToCamPosition().getY()).plus(cameraToObject);
    return robotToObject;
  }


  public Translation2d getOriginToObject(){
    robotToObject = getRobotToObject().rotateBy(getRobotAngle.get());
    OriginToObject = robotToObject.plus(robotCurrentPose.get().getTranslation());
    return OriginToObject;
  }
}
