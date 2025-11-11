// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testMotors.talonSRX.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Motors.TalonSRXMotor;
import frc.robot.testMotors.talonSRX.Constants;

public class Motor extends SubsystemBase {
  private TalonSRXMotor talonMotor;

  /** Creates a new motor. */
  public Motor() {
    talonMotor = new TalonSRXMotor(Constants.CONFIG);
  }

  public TalonSRXMotor getMotor(){
    return talonMotor;
  }
}
