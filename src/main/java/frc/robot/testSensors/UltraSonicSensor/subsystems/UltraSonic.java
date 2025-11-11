// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testSensors.ultraSonicSensor.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Sensors.UltraSonicSensor;
import frc.robot.testSensors.ultraSonicSensor.*;

public class UltraSonic extends SubsystemBase {
  UltraSonicSensor ultraSonicSensor;
  /** Creates a new subsystem. */
  public UltraSonic() {
    ultraSonicSensor = new UltraSonicSensor(Constants.ULTRA_SONIC_SENSOR_CONFIG);
  }
  
  public UltraSonicSensor getUltraSonicSensor(){
    return ultraSonicSensor;
  }
}
