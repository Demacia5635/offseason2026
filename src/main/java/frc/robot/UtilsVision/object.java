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
import frc.robot.UtilsVision.Cemera;

public class object extends SubsystemBase {
  private Translation2d robotToTag;
  private Translation2d cameraToTag;

  private NetworkTable Table;
  private double wantedPip = 0;


  private double camToObjectYaw;
  private double camToObjectPitch;

  
  private Supplier<Rotation2d> getRobotAngle;
  private Supplier<Pose2d> currentPose;


  private Cemera camera;
  public object(Cemera camera, Supplier<Rotation2d> getRobotAngle,Supplier<Pose2d> currentPose) {
    this.getRobotAngle = getRobotAngle;
    this.currentPose = currentPose;

    this.camera = camera;
    Table = NetworkTableInstance.getDefault().getTable(camera.getTableName());
  }

  @Override
  public void periodic() {
    if(Table.getEntry("tv").getDouble(0.0) != 0){
      
    }
    // This method will be called once per scheduler run
  }
}
