// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testSensors.cancoder.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.testSensors.cancoder.Constants;



public class Cancoder extends SubsystemBase {
  frc.demacia.utils.Sensors.Cancoder cancoder;
  /** Creates a new Cancoder. */
  public Cancoder() {
    cancoder = new frc.demacia.utils.Sensors.Cancoder(Constants.CANCODER_CONFIG);
    

  }
  public frc.demacia.utils.Sensors.Cancoder getCancoder(){
    return cancoder;
  }

  }






