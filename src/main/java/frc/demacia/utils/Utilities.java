package frc.demacia.utils;

/**
 * Static utility functions for common mathematical operations.
 * 
 * <p>Provides fast, optimized implementations of frequently-used functions.</p>
 */
public class Utilities {

    /**
     * Clamps a value between minimum and maximum bounds.
     * 
     * @param value Value to clamp
     * @param min Minimum allowed value
     * @param max Maximum allowed value
     * @return Clamped value
     */
    public static double clamp(double value, double min, double max) {
        return value < min ? min : value > max ? max : value;
    }

    /**
     * Applies deadband to a value.
     * 
     * <p>Values within ±deadband are set to zero. Useful for joystick inputs.</p>
     * 
     * @param value Input value
     * @param deadband Deadband threshold (positive)
     * @return 0 if |value| < deadband, otherwise value
     */
    public static double deadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0 : value;
    }

    /**
     * Applies deadband to a value.
     * 
     * <p>Values within ±deadband are set to zero. Useful for joystick inputs.</p>
     * 
     * @param value Input value
     * @param deadband Deadband threshold (positive)
     * @return 0 if |value| < deadband, otherwise value
     */
    public static double signumWithDeadband(double value, double deadband) {
        return value > deadband ? 1 : value < -deadband ? -1 : 0;
    }

    /**
     * Fast hypotenuse calculation (magnitude of 2D vector).
     * 
     * <p>Equivalent to Math.sqrt(x² + y²) but may be optimized.</p>
     * 
     * @param x X component
     * @param y Y component
     * @return Magnitude sqrt(x² + y²)
     */
    public static double hypot(double x, double y) {
      return Math.sqrt(x*x + y*y);
    }

    /**
     * Calculates angle from 2D translation components.
     * 
     * <p>Returns angle in radians using atan2 with normalization.</p>
     * 
     * @param x X component
     * @param y Y component
     * @return Angle in radians (-π to π), or 0 if magnitude < 1e-6
     */
    public static double angleFromTranslation2d(double x, double y) {
        double magnitude = hypot(x, y);
        if (magnitude > 1e-6) {
            return Math.atan2(y / magnitude, x / magnitude);
        } 
        return 0;
    }
}
