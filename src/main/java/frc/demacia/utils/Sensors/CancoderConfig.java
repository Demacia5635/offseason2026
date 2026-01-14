// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.Sensors;

import com.ctre.phoenix6.CANBus;

/** Add your docs here. */
public class CancoderConfig extends BaseSensorConfig<CancoderConfig>{
    public double offset = 0;
    /** 
     * Constructor
     * @param id - canbus ID
     * @param canbus - Name of canbus
     * @param name - name of Cancoder for logging 
     */
    public CancoderConfig(int id, CANBus canbus, String name) {
        super(id, canbus, name);
        sensorType = Cancoder.class;
    }

    public CancoderConfig withOffset(double offset) {
        this.offset = offset;
        return this;
    }
}