package frc.Demacia.utils;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.Demacia.utils.constants.UtilsContants;

public class DriverUtils {
    
    public enum JoystickSide { LeftX, LeftY, RightX, RightY;
        double value(XboxController controller) {
            switch (this) {
                case LeftX:
                    return controller.getLeftX();
                case LeftY:
                    return -controller.getLeftY();
                case RightX:
                    return controller.getRightX();
                case RightY:
                    return -controller.getRightY();
                default:
                    return 0;
            }
    
        }
        double value(CommandXboxController controller) {
            return value(controller.getHID());
        };
        
        double value(PS5Controller controller) {
            switch (this) {
                case LeftX:
                    return controller.getLeftX();
                case LeftY:
                    return -controller.getLeftY();
                case RightX:
                    return controller.getRightX();
                case RightY:
                    return -controller.getRightY();
                default:
                    return 0;
            }
    
        }
        double value(CommandPS5Controller controller) {
            return value(controller.getHID());
        };
    
    }
    
    public static double getNormalized(double value, double deadband, boolean power) {
        return  value < -deadband ? (power ? -value*value : value):
                value > deadband ? (power ? value*value : value) :
                0;
    }
    
    public static double getNormalized(double value, double deadband) {
        return getNormalized(value, deadband, true);
    }
    public static double getNormalized(double value) {
        return getNormalized(value, UtilsContants.ControllerConstants.XBOX_STICK_DEADBAND, true);
    }

    public static double getJSvalue(XboxController controller, JoystickSide side, double deadband, boolean usePower) {
        return getNormalized(side.value(controller), deadband, usePower);
    }
    public static double getJSvalue(XboxController controller, JoystickSide side, double deadband) {
        return getJSvalue(controller, side, deadband, true);
    }
    public static double getJSvalue(XboxController controller, JoystickSide side) {
        return getJSvalue(controller, side, UtilsContants.ControllerConstants.XBOX_STICK_DEADBAND, true);
    }

    public static double getJSvalue(CommandXboxController controller, JoystickSide side, double deadband, boolean usePower) {
        return getJSvalue(controller.getHID(),side,deadband,usePower);
    }
    public static double getJSvalue(CommandXboxController controller, JoystickSide side, double deadband) {
        return getJSvalue(controller.getHID(), side, deadband, true);
    }
    public static double getJSvalue(CommandXboxController controller, JoystickSide side) {
        return getJSvalue(controller.getHID(), side, UtilsContants.ControllerConstants.XBOX_STICK_DEADBAND, true);
    }
    public static double getTriggerValue(CommandXboxController controller) {
        return getNormalized(controller.getLeftTriggerAxis() - controller.getRightTriggerAxis(), UtilsContants.ControllerConstants.XBOX_TRIGGER_DEADBAND);
    }
    public static double getJSvalue(PS5Controller controller, JoystickSide side, double deadband, boolean usePower) {
        return getNormalized(side.value(controller), deadband, usePower);
    }
    public static double getJSvalue(PS5Controller controller, JoystickSide side, double deadband) {
        return getJSvalue(controller, side, deadband, true);
    }
    public static double getJSvalue(PS5Controller controller, JoystickSide side) {
        return getJSvalue(controller, side, UtilsContants.ControllerConstants.PS5_STICK_DEADBAND, true);
    }

    public static double getJSvalue(CommandPS5Controller controller, JoystickSide side, double deadband, boolean usePower) {
        return getJSvalue(controller.getHID(),side,deadband,usePower);
    }
    public static double getJSvalue(CommandPS5Controller controller, JoystickSide side, double deadband) {
        return getJSvalue(controller.getHID(), side, deadband, true);
    }
    public static double getJSvalue(CommandPS5Controller controller, JoystickSide side) {
        return getJSvalue(controller.getHID(), side, UtilsContants.ControllerConstants.PS5_STICK_DEADBAND, true);
    }
    public static double getTriggerValue(CommandPS5Controller controller) {
        return getNormalized(controller.getL2Axis() - controller.getR2Axis(), UtilsContants.ControllerConstants.PS5_TRIGGER_DEADBAND);
    }

}
