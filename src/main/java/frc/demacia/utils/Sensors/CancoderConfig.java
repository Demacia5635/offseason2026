// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.Sensors;

import com.ctre.phoenix6.CANBus;

import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;

/**
 * Configuration for CTRE CANcoder absolute encoder.
 * 
 * <p><b>Calibration Process:</b></p>
 * <ol>
 *   <li>Physically align mechanism to known position</li>
 *   <li>Read raw encoder value</li>
 *   <li>Set offset to that value</li>
 *   <li>Now encoder reads 0 at aligned position</li>
 * </ol>
 * 
 * <p><b>Example:</b></p>
 * <pre>
 * // Swerve module encoder - wheel points forward at offset
 * CancoderConfig config = new CancoderConfig(15, CANBus.CANivore, "FrontLeftSteer")
 *     .withOffset(-0.523)  // Measured when wheel forward
 *     .withInvert(false);
 * </pre>
 */
public class CancoderConfig extends BaseSensorConfig<CancoderConfig>{
    public double offset = 0;
    
    /**
     * Creates CANcoder configuration.
     * 
     * @param id CAN bus ID (1-63)
     * @param canbus CANBus instance (typically CANivore for swerve)
     * @param name Descriptive name for logging
     */
    public CancoderConfig(int id, Canbus canbus, String name) {
        super(id, canbus, name);
        sensorType = Cancoder.class;
    }

    /**
     * Sets the zero offset for calibration.
     * 
     * <p>Offset is subtracted from raw reading so calibrated position reads zero.</p>
     * 
     * @param offset Offset in rotations (will be converted to radians internally)
     * @return this config for method chaining
     */
    public CancoderConfig withOffset(double offset) {
        this.offset = offset;
        return this;
    }
}