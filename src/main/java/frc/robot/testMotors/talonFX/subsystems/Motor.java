// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testMotors.talonFX.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Motors.TalonMotor;
import frc.robot.testMotors.talonFX.Constants;

public class Motor extends SubsystemBase {
  private TalonMotor talonMotor;

  /** Creates a new motor. */
  public Motor() {
    talonMotor = new TalonMotor(Constants.CONFIG);
  }

  public TalonMotor getMotor(){
    return talonMotor;
  }
}
