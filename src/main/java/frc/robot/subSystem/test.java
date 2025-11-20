// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subSystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Motors.TalonMotor;
import frc.robot.Constants;

public class test extends SubsystemBase {

  Constants constants;
  
  private TalonMotor motor;

  /** Creates a new test. */
  public test() {
    motor = new TalonMotor(constants.config);
  }

  public void test(){
    long start = System.currentTimeMillis();
    motor.setEncoderPosition(0);
    motor.setAngle(90);
    long stop = System.currentTimeMillis() - start;
    motor.setAngle(0);
    System.out.println(stop);
  }

  public void stop(){
    motor.setDuty(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
