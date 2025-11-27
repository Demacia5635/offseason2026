// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subSystem;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Motors.TalonMotor;
import frc.robot.Constants;

public class test extends SubsystemBase {
  
  private TalonMotor motor;

  private StatusSignal<AngularAcceleration> accelerationSignal;
  private StatusSignal<AngularVelocity> velocitySignal;
  private StatusSignal<Angle> Angle;

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

  public StatusSignal<AngularAcceleration> Acceleration(StatusSignal<AngularAcceleration> accelerationSignal){
    this.accelerationSignal = accelerationSignal;
    accelerationSignal = motor.getAcceleration();
    return accelerationSignal;
  }

  public StatusSignal<AngularVelocity> velocity (StatusSignal<AngularVelocity> velocitySignal){
    this.velocitySignal = velocitySignal;
    velocitySignal = motor.getVelocity();
    return velocitySignal;
  }

  public StatusSignal<Angle> position(StatusSignal<Angle> Angle){
    this.Angle = Angle;
    Angle = motor.getPosition();
    return Angle;
  }

  public void setEnc(double Position){
    motor.setEncoderPosition(Position);
  }

  public void print(){
    System.out.println(accelerationSignal);
    System.out.println(velocitySignal);
    System.out.println(Angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
