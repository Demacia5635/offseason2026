// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testMotors.sparkFlex.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Motors.SparkFlexMotor;
import frc.robot.testMotors.sparkFlex.Constants;

public class Motor extends SubsystemBase {
  private SparkFlexMotor sparkMotor;

  /** Creates a new motor. */
  public Motor() {
    sparkMotor = new SparkFlexMotor(Constants.CONFIG);
  }

  public SparkFlexMotor getMotor(){
    return sparkMotor;
  }
}
