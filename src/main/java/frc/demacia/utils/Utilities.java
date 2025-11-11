package frc.demacia.utils;

public class Utilities {

    public static double clamp(double value, double min, double max) {
        return value < min ? min : value > max ? max : value;
    }
    public static double deadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0 : value;
    }
    public static double signumWithDeadband(double value, double deadband) {
        return value > deadband ? 1 : value < -deadband ? -1 : 0;
    }

    public static double hypot(double x, double y) {
      return Math.sqrt(x*x + y*y);
    }

    public static double angleFromTranslation2d(double x, double y) {
        double magnitude = hypot(x, y);
        if (magnitude > 1e-6) {
            return Math.atan2(y / magnitude, x / magnitude);
        } 
        return 0;
    }
}
