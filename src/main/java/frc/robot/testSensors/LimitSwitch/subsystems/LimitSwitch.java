// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testSensors.LimitSwitch.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.testSensors.LimitSwitch.Constants;



public class LimitSwitch extends SubsystemBase {
  frc.demacia.utils.Sensors.LimitSwitch limitSwitch;
  /** Creates a new Pigeon. */
  public LimitSwitch() {
    limitSwitch = new frc.demacia.utils.Sensors.LimitSwitch(Constants.LIMIT_SWITCH_CONFIG);
    

  }
  public frc.demacia.utils.Sensors.LimitSwitch getLimitSwitch(){
    return limitSwitch;
  }

  }






