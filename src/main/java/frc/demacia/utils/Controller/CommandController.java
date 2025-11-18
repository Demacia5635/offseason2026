package frc.demacia.utils.Controller;  

import edu.wpi.first.hal.FRCNetComm.tResourceType;

import static frc.demacia.utils.Controller.ControllerConstants.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.demacia.utils.Utilities;
import frc.demacia.utils.Log.LogManager;

/**
 * Unified controller wrapper supporting Xbox and PS5 controllers.
 * 
 * <p>Provides consistent button/axis mapping across controller types
 * with automatic deadband handling.</p>
 * 
 * <p><b>Features:</b></p>
 * <ul>
 *   <li>Automatic deadband application (configurable)</li>
 *   <li>Consistent naming across controller types</li>
 *   <li>Trigger support (both digital and analog)</li>
 *   <li>Stick movement detection</li>
 * </ul>
 * 
 * <p><b>Example:</b></p>
 * <pre>
 * CommandController driver = new CommandController(0, ControllerType.kXbox);
 * 
 * // Same code works for both Xbox and PS5
 * driver.downButton().onTrue(Commands.runOnce(() -> shoot()));
 * double speed = -driver.getLeftY();  // Forward/backward
 * double turn = driver.getRightX();   // Left/right
 * </pre>
 */
public class CommandController extends CommandGenericHID{

    /**
     * Enum defining supported controller types.
     */
    public enum ControllerType {
        kXbox,
        kPS5;
    }

    private final ControllerType controllerType;

    /**
     * Creates a new controller wrapper.
     * 
     * @param port USB port (0-5)
     * @param controllerType Type of controller connected
     */
    public CommandController(int port, ControllerType controllerType) {
        super(port);
        this.controllerType = controllerType;
        HAL.report(tResourceType.kResourceType_Controller, port + 1, 0, "Driver Controller");
    }

    /**
     * Top face button (Y on Xbox, Triangle on PS5).
     * 
     * @return Trigger for command binding
     */
    public Trigger upButton() {
        switch(controllerType) {
            case kXbox:
                return button(XboxController.Button.kY.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kTriangle.value , CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    /**
     * Left face button (X on Xbox, Square on PS5).
     * 
     * @return Trigger for command binding
     */
    public Trigger leftButton() {
        switch(controllerType) {
            case kXbox:
                return button(XboxController.Button.kX.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kSquare.value , CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    /**
     * Bottom face button (A on Xbox, Cross on PS5).
     * 
     * @return Trigger for command binding
     */
    public Trigger downButton() {
        switch(controllerType) {
            case kXbox:
                return button(XboxController.Button.kA.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kCross.value , CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    /**
     * Right face button (B on Xbox, Circle on PS5).
     * 
     * @return Trigger for command binding
     */
    public Trigger rightButton() {
        switch (controllerType) {
            case kXbox:
                return button(XboxController.Button.kB.value, CommandScheduler.getInstance().getDefaultButtonLoop());
        
            case kPS5:
                return button(PS5Controller.Button.kCircle.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    /**
     * Left bumper (LB on Xbox, L1 on PS5).
     * 
     * @return Trigger for command binding
     */
    public Trigger leftBumper() {
        switch (controllerType) {
            case kXbox:
                return button(XboxController.Button.kLeftBumper.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kL1.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    /**
     * Right bumper (RB on Xbox, R1 on PS5).
     * 
     * @return Trigger for command binding
     */
    public Trigger rightBumper() {
        switch(controllerType) {
            case kXbox:
                return button(XboxController.Button.kRightBumper.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kR1.value , CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    /**
     * Left stick click (LS on Xbox, L3 on PS5).
     * 
     * @return Trigger for command binding
     */
    public Trigger leftStick() {
        switch (controllerType) {
            case kXbox:
                return button(XboxController.Button.kLeftStick.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kL3.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    /**
     * Right stick click (RS on Xbox, R3 on PS5).
     * 
     * @return Trigger for command binding
     */
    public Trigger rightStick() {
        switch(controllerType) {
            case kXbox:
                return button(XboxController.Button.kRightStick.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kR3.value , CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    /**
     * Right menu button (Back on Xbox, Options on PS5).
     * 
     * @return Trigger for command binding
     */
    public Trigger rightSetting() {
        switch (controllerType) {
            case kXbox:
                return button(XboxController.Button.kBack.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kOptions.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    /**
     * Left menu button (Start on Xbox, Create on PS5).
     * 
     * @return Trigger for command binding
     */
    public Trigger leftSettings() {
        switch (controllerType) {
            case kXbox:
                return button(XboxController.Button.kStart.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kCreate.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    /**
     * Gets left stick X-axis value with deadband.
     * 
     * @return Value from -1.0 (left) to 1.0 (right), 0 when in deadband
     */
    public double getLeftX() {
        switch (controllerType) {
            case kXbox:
                return Utilities.deadband(getRawAxis(XboxController.Axis.kLeftX.value), XBOX_STICK_DEADBAND);
            case kPS5:
                return Utilities.deadband(getRawAxis(PS5Controller.Axis.kLeftX.value), PS5_STICK_DEADBAND);
            default:
                return 0;
        }
    }

    /**
     * Gets left stick Y-axis value with deadband.
     * 
     * <p><b>Note:</b> Returns positive for up, negative for down (inverted from raw)</p>
     * 
     * @return Value from -1.0 (down) to 1.0 (up), 0 when in deadband
     */
    public double getLeftY() {
        switch (controllerType) {
            case kXbox:
                return Utilities.deadband(getRawAxis(XboxController.Axis.kLeftY.value), XBOX_STICK_DEADBAND);
            case kPS5:
                return Utilities.deadband(getRawAxis(PS5Controller.Axis.kLeftY.value), PS5_STICK_DEADBAND);
            default:
                return 0;
        }
    }

    /**
     * Gets right stick X-axis value with deadband.
     * 
     * @return Value from -1.0 (left) to 1.0 (right), 0 when in deadband
     */
    public double getRightX() {
        switch (controllerType) {
            case kXbox:
                return Utilities.deadband(getRawAxis(XboxController.Axis.kRightX.value), XBOX_STICK_DEADBAND);
            case kPS5:
                return Utilities.deadband(getRawAxis(PS5Controller.Axis.kRightX.value), PS5_STICK_DEADBAND);
            default:
                return 0;
        }
    }

    /**
     * Gets right stick Y-axis value with deadband.
     * 
     * @return Value from -1.0 (down) to 1.0 (up), 0 when in deadband
     */
    public double getRightY() {
        switch (controllerType) {
            case kXbox:
                return Utilities.deadband(getRawAxis(XboxController.Axis.kRightY.value), XBOX_STICK_DEADBAND);
            case kPS5:
                return Utilities.deadband(getRawAxis(PS5Controller.Axis.kRightY.value), PS5_STICK_DEADBAND);
            default:
                return 0;
        }
    }

    /**
     * Gets left trigger analog value with deadband.
     * 
     * <p>Normalized to 0.0-1.0 for both controller types.</p>
     * 
     * @return Value from 0.0 (released) to 1.0 (fully pressed)
     */
    public double getLeftTrigger() {
        switch (controllerType) {
            case kXbox:
                return Utilities.deadband(getRawAxis(XboxController.Axis.kLeftTrigger.value), XBOX_TRIGGER_DEADBAND);
            case kPS5:
                return Utilities.deadband((getRawAxis(PS5Controller.Axis.kL2.value) + 1) / 2, PS5_TRIGGER_DEADBAND);
            default:
                return 0;
        }
    }

    /**
     * Gets right trigger analog value with deadband.
     * 
     * @return Value from 0.0 (released) to 1.0 (fully pressed)
     */
    public double getRightTrigger() {
        switch (controllerType) {
            case kXbox:
                return Utilities.deadband(getRawAxis(XboxController.Axis.kRightTrigger.value), XBOX_TRIGGER_DEADBAND);
            case kPS5:
                return Utilities.deadband((getRawAxis(PS5Controller.Axis.kR2.value) + 1) / 2, PS5_TRIGGER_DEADBAND);
            default:
                return 0;
        }
    }

    /**
     * Creates a trigger for left stick X-axis threshold.
     * 
     * @param threshold Activation threshold (-1.0 to 1.0)
     * @return Trigger that activates when axis exceeds threshold
     */
    public Trigger getLeftX(double threshold) {
        switch (controllerType) {
            case kXbox:
                return axisGreaterThan(XboxController.Axis.kLeftX.value, threshold);
            case kPS5:
                return axisGreaterThan(PS5Controller.Axis.kLeftX.value, threshold);
            default:
                return null;
        }
    }

    /**
     * Creates a trigger for left stick Y-axis threshold.
     * 
     * @param threshold Activation threshold (-1.0 to 1.0)
     * @return Trigger that activates when axis exceeds threshold
     */
    public Trigger getLeftY(double threshold) {
        switch (controllerType) {
            case kXbox:
                return axisGreaterThan(XboxController.Axis.kLeftY.value, threshold);
            case kPS5:
                return axisGreaterThan(PS5Controller.Axis.kLeftY.value, threshold);
            default:
                return null;
        }
    }

    /**
     * Creates a trigger for right stick X-axis threshold.
     * 
     * @param threshold Activation threshold (-1.0 to 1.0)
     * @return Trigger that activates when axis exceeds threshold
     */
    public Trigger getRightX(double threshold) {
        switch (controllerType) {
            case kXbox:
                return axisGreaterThan(XboxController.Axis.kRightX.value, threshold);
            case kPS5:
                return axisGreaterThan(PS5Controller.Axis.kRightX.value, threshold);
            default:
                return null;
        }
    }

    /**
     * Creates a trigger for right stick Y-axis threshold.
     * 
     * @param threshold Activation threshold (-1.0 to 1.0)
     * @return Trigger that activates when axis exceeds threshold
     */
    public Trigger getRightY(double threshold) {
        switch (controllerType) {
            case kXbox:
                return axisGreaterThan(XboxController.Axis.kRightY.value, threshold);
            case kPS5:
                return axisGreaterThan(PS5Controller.Axis.kRightY.value, threshold);
            default:
                return null;
        }
    }

    /**
     * Creates a trigger for left trigger threshold.
     * 
     * @param threshold Activation threshold (0.0 to 1.0)
     * @return Trigger that activates when trigger exceeds threshold
     */
    public Trigger getLeftTrigger(double threshold) {
        switch (controllerType) {
            case kXbox:
                return axisGreaterThan(XboxController.Axis.kLeftTrigger.value, threshold);
            case kPS5:
                return axisGreaterThan(PS5Controller.Axis.kL2.value, threshold * 2  - 1);
            default:
                return null;
        }
    }

    /**
     * Creates a trigger for right trigger threshold.
     * 
     * @param threshold Activation threshold (0.0 to 1.0)
     * @return Trigger that activates when trigger exceeds threshold
     */
    public Trigger getRightTrigger(double threshold) {
        switch (controllerType) {
            case kXbox:
                return axisGreaterThan(XboxController.Axis.kRightTrigger.value, threshold);
            case kPS5:
                return axisGreaterThan(PS5Controller.Axis.kR2.value, threshold * 2 - 1);
            default:
                return null;
        }
    }

    /**
     * PS5-only: PlayStation button.
     * 
     * @return Trigger for PS button, or null if Xbox controller
     */
    public Trigger getPS() {
        switch (controllerType) {
            case kXbox:
                LogManager.log("Xbox controller does not have PS button", AlertType.kError);
                return null;
            case kPS5:
                return button(PS5Controller.Button.kPS.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    /**
     * PS5-only: Touchpad button.
     * 
     * @return Trigger for touchpad, or null if Xbox controller
     */
    public Trigger getTouchPad() {
        switch (controllerType) {
            case kXbox:
                LogManager.log("Xbox controller does not have touchpad", AlertType.kError);
                return null;        
            case kPS5:
                return button(PS5Controller.Button.kTouchpad.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    /**
     * Creates a trigger that activates when left stick is moved.
     * 
     * <p>Useful for detecting any stick input without checking specific directions.</p>
     * 
     * @return Trigger that activates when stick exceeds 0.3 in any direction
     */
    public Trigger getLeftStickMove() {
        return new Trigger(()-> Math.abs(getLeftX()) >= 0.3 || Math.abs(getLeftY()) >= 0.3);
    }

    /**
     * Creates a trigger that activates when right stick is moved.
     * 
     * @return Trigger that activates when stick is moved in any direction
     */
    public Trigger getRightStickMove() {
        return new Trigger(()-> getRightX() != 0 || getRightY() != 0);
    }
}