package frc.demacia.utils.Motors;

/**
 * Unified interface for all motor controller types.
 * 
 * <p>Provides consistent control methods regardless of underlying hardware.
 * Implementations handle vendor-specific details internally.</p>
 * 
 * <p><b>Control modes available:</b></p>
 * <ul>
 *   <li>Duty cycle (percent output)</li>
 *   <li>Voltage control</li>
 *   <li>Velocity control (closed-loop)</li>
 *   <li>Position control (closed-loop)</li>
 *   <li>Motion profiled control (smooth acceleration)</li>
 *   <li>Angle control (with automatic wrap-around)</li>
 * </ul>
 */
public interface MotorInterface {

    /**
     * Gets the motor's name as specified in config.
     * 
     * @return Motor name for logging/debugging
     */
    String name();

    /**
     * Changes the active PID/FF slot.
     * 
     * <p>Allows switching between different control strategies on-the-fly.
     * Slot must have been configured with withPID().</p>
     * 
     * @param slot Slot number (0-2)
     */
    void changeSlot(int slot);
    /**
     * Sets the motor's neutral mode (brake vs coast).
     * 
     * @param isBrake true for brake mode, false for coast mode
     */
    void setNeutralMode(boolean isBrake);
    /**
     * Sets motor duty cycle (percent output).
     * 
     * <p><b>Open-loop control</b> - no feedback, simple voltage control.
     * Useful for manual control, testing, or when precise control isn't needed.</p>
     * 
     * @param power Output power from -1.0 (full reverse) to 1.0 (full forward)
     */
    void setDuty(double power);
    /**
     * Sets motor voltage directly.
     * 
     * <p><b>Open-loop control</b> - provides consistent behavior across battery voltages.
     * Voltage compensation is applied automatically.</p>
     * 
     * @param voltage Output voltage (typically -12V to +12V)
     */
    void setVoltage(double voltage);
    /**
     * Sets motor velocity with closed-loop control and optional feed-forward.
     * 
     * <p><b>Use case:</b> Flywheels, conveyors, continuous motion</p>
     * 
     * <p>The motor will actively maintain the target velocity using PID control.
     * Additional feed-forward helps overcome friction and improve response.</p>
     * 
     * @param velocity Target velocity in configured units per second
     * @param feedForward Additional voltage to add (typically from kS calculation)
     */
    void setVelocity(double velocity, double feedForward);
    /**
     * Sets motor velocity with closed-loop control.
     * Automatically applies kS feed-forward based on direction.
     * 
     * @param velocity Target velocity in configured units per second
     */
    void setVelocity(double velocity);
    /**
     * Sets target position using motion profiling (smooth acceleration).
     * 
     * <p><b>Use case:</b> Arms, elevators, precise positioning</p>
     * 
     * <p>The motor follows a trapezoidal or S-curve profile to reach the target position
     * smoothly, respecting max velocity, acceleration, and jerk constraints.</p>
     * 
     * <p><b>Requires:</b> withMotionParam() must be called in config</p>
     * 
     * @param position Target position in configured units
     * @param feedForward Additional voltage to add (e.g., for gravity compensation)
     */
    void setMotion(double position, double feedForward);
    /**
     * Sets target position using motion profiling.
     * Automatically applies kS feed-forward based on direction.
     * 
     * @param position Target position in configured units
     */
    void setMotion(double position);
    /**
     * Sets target angle with automatic wrap-around handling.
     * 
     * <p><b>Use case:</b> Swerve modules, turrets, rotating mechanisms</p>
     * 
     * <p>Automatically chooses the shortest path, accounting for wrap-around.
     * For example, going from 350° to 10° will rotate 20° forward, not 340° backward.</p>
     * 
     * <p><b>Requires:</b> Motor must be configured with withRadiansMotor() or withDegreesMotor()</p>
     * 
     * @param angle Target angle in radians or degrees (based on config)
     * @param feedForward Additional voltage to add
     */
    void setAngle(double angle, double feedForward);
    /**
     * Sets target angle with automatic wrap-around handling.
     * Automatically applies kS feed-forward.
     * 
     * @param angle Target angle in radians or degrees (based on config)
     */
    void setAngle(double angle);
    /**
     * Sets target position with voltage-based control (no motion profiling).
     * 
     * <p>Simpler than motion profiling but less smooth. The motor will attempt
     * to reach the position as quickly as possible.</p>
     * 
     * @param position Target position in configured units
     * @param feedForward Additional voltage to add
     */
    void setPositionVoltage(double position, double feedForward);
    /**
     * Sets target position with voltage-based control.
     * 
     * @param position Target position in configured units
     */
    void setPositionVoltage(double position);
    /**
     * Sets velocity with advanced feed-forward calculation.
     * 
     * <p>Applies kV² term for velocity-squared compensation (air resistance).</p>
     * 
     * @param velocity Target velocity in configured units per second
     */
    void setVelocityWithFeedForward(double velocity);
    /**
     * Sets motion-profiled position with advanced feed-forward.
     * 
     * <p>Applies sine-based compensation for gravitational effects on rotating arms.</p>
     * 
     * @param position Target position in configured units
     */
    void setMotionWithFeedForward(double velocity);

    /**
     * Gets the current control mode as a human-readable string.
     * 
     * @return Control mode name (e.g., "Velocity", "Motion", "Duty Cycle")
     */
    String getCurrentControlMode();
    /**
     * Gets the current closed-loop setpoint.
     * 
     * @return Target value being tracked (velocity or position depending on mode)
     */
    double getCurrentClosedLoopSP();
    /**
     * Gets the current closed-loop error.
     * 
     * @return Difference between setpoint and actual value
     */
    double getCurrentClosedLoopError();
    /**
     * Gets the current position from the encoder.
     * 
     * @return Current position in configured units
     */
    double getCurrentPosition();
    /**
     * Gets the current angle with wrap-around applied.
     * 
     * <p><b>Radians motor:</b> Returns angle modulated to [-π, π]</p>
     * <p><b>Degrees motor:</b> Returns angle modulated to [-180°, 180°]</p>
     * 
     * @return Current angle in configured units
     */
    double getCurrentAngle();
    /**
     * Gets the current velocity from the encoder.
     * 
     * @return Current velocity in configured units per second
     */
    double getCurrentVelocity();
    /**
     * Gets the current acceleration (calculated from velocity).
     * 
     * @return Current acceleration in configured units per second²
     */
    double getCurrentAcceleration();
    /**
     * Gets the voltage currently being applied to the motor.
     * 
     * @return Applied voltage in volts
     */
    double getCurrentVoltage();
    /**
     * Gets the current being drawn by the motor.
     * 
     * @return Current draw in amps
     */
    double getCurrentCurrent();

    /**
     * Checks motor controller for faults and logs them.
     * 
     * <p>Should be called periodically (e.g., in periodic() method) to catch issues.
     * Logs faults to console and telemetry.</p>
     */
    void checkElectronics();

    /**
     * Resets the encoder position to a specific value.
     * 
     * <p>Useful for zeroing mechanisms or setting absolute positions.</p>
     * 
     * @param position New position value in configured units
     */
    void setEncoderPosition(double position);

    /**
     * Creates a hot-reload widget for tuning PID parameters.
     * 
     * <p>Allows real-time tuning through dashboard without redeploying code.
     * Changes are applied immediately to the motor controller.</p>
     * 
     * @param slot PID slot to display (0-2)
     */
    void showConfigPIDFSlotCommand(int slot);
    /**
    * Creates a hot-reload widget for tuning motion profile parameters.
    * 
    * <p>Allows real-time tuning of maxVelocity, maxAcceleration, and maxJerk.</p>
    */
    void showConfigMotionVelocitiesCommand();

}
