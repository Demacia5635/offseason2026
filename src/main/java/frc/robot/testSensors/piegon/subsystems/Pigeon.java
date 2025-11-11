// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testSensors.piegon.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.testSensors.piegon.Constants;



public class Pigeon extends SubsystemBase {
  frc.demacia.utils.Sensors.Pigeon pigeon;
  /** Creates a new Pigeon. */
  public Pigeon() {
    pigeon= new frc.demacia.utils.Sensors.Pigeon(Constants.PIGEON_CONFIG);
    

  }
  public frc.demacia.utils.Sensors.Pigeon getPigeon(){
    return pigeon;
  }

  }






