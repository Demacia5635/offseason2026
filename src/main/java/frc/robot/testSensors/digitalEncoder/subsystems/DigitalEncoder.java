// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testSensors.digitalEncoder.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.testSensors.digitalEncoder.Constants;



public class DigitalEncoder extends SubsystemBase {
  frc.demacia.utils.Sensors.DigitalEncoder digitalEncoder;
  /** Creates a new Pigeon. */
  public DigitalEncoder() {
    digitalEncoder= new frc.demacia.utils.Sensors.DigitalEncoder(Constants.DIGITAL_ENCODER_CONFIG);
    

  }
  public frc.demacia.utils.Sensors.DigitalEncoder getDigitalEncoder(){
    return digitalEncoder;
  }

  }






