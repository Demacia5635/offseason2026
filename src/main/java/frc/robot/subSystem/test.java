// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subSystem;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.Motors.TalonMotor;
import frc.robot.Constants;

public class test extends SubsystemBase {
  
  private TalonMotor motor;

  Timer timer;


  /** Creates a new test. */
  public test() {
    motor = new TalonMotor(Constants.config);
    timer = new Timer();
    timer.start();
    SmartDashboard.putData("motor", motor);
    SmartDashboard.putData("motor/setBrake", new InstantCommand(()-> setNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData("motor/setCoast", new InstantCommand(()-> setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putNumber("vel", getVelosty());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    
    builder.addDoubleProperty("velosity", () -> getVelosty(), null);
  }

  public void setNeutralMode(boolean isBrake) {
    motor.setNeutralMode(isBrake);
  }

  public double getang(){
    return motor.getCurrentAngle();
  }

  public double getVelosty(){
    return motor.getCurrentVelocity();
  }

  public void setPow(double pow){
    motor.set(pow);
  }
  public void stop(){
    motor.setDuty(0);
  }

  public void setEnc(double Position){
    motor.setEncoderPosition(Position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getVelosty()>0.5 && timer.get() > 0.05){
      LogManager.log("velosry: "+ getVelosty() + " time: " + Timer.getFPGATimestamp());
      timer.reset();
    }
  }
}
