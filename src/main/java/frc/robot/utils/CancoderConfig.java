// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.CANBus;

/** Add your docs here. */
public class CancoderConfig {
    public int id;                  // Canbus ID
    public CANBus canbus;           // Canbus name
    public String name; 
    public double offset = 0;
    public boolean inverted = false; // if to invert cancoderr
    /** 
     * Constructor
     * @param id - canbus ID
     * @param canbus - Name of canbus
     * @param name - name of Cancoder for logging
     */
    public CancoderConfig(int id, CANBus canbus, String name) {
        this.id = id;
        this.canbus = canbus;
        this.name = name;
    }

    public CancoderConfig withOffset(double offset) {
        this.offset = offset;
        return this;
    }

    /** 
     * @param invert
     * @return CancoderConfig
     */
    public CancoderConfig withInvert(boolean invert) {
        this.inverted = invert;
        return this;
    }
}
