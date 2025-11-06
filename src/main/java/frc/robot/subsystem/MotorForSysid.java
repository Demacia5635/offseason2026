// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Motors.BaseMotorConfig;
import frc.demacia.utils.Motors.MotorInterface;
import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.TalonMotor;
import frc.robot.Constants;

public class MotorForSysid extends SubsystemBase {
  TalonMotor motor;

  /** Creates a new MotorForSysid. */
  public MotorForSysid() {
    motor = new TalonMotor(Constants.motorConstants.CONFIG);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power){
    motor.setDuty(power);
  }
}
