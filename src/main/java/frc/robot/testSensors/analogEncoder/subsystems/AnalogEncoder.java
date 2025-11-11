// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testSensors.analogEncoder.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.testSensors.analogEncoder.Constants;



public class AnalogEncoder extends SubsystemBase {
  frc.demacia.utils.Sensors.AnalogEncoder analogEncoder;
  /** Creates a new AnalogEncoder. */
  public AnalogEncoder() {
    analogEncoder = new frc.demacia.utils.Sensors.AnalogEncoder(Constants.ANALOG_ENCODER_CONFIG);
    

  }
  public frc.demacia.utils.Sensors.AnalogEncoder getAnalogEncoder(){
    return analogEncoder;
  }

  }






