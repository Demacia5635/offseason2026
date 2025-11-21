// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subSystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Motors.TalonMotor;
import frc.robot.Constants;

public class test extends SubsystemBase {
  
  private TalonMotor motor;

  /** Creates a new test. */
  public test() {
    motor = new TalonMotor(Constants.config);
    SmartDashboard.putData("motor", motor);
  }

  public double getang(){
    return motor.getCurrentAngle();
  }
  public void setPow(double pow){
    motor.set(pow);
  }
  public void stop(){
    motor.setDuty(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
