package frc.demacia.utils.Motors;

import edu.wpi.first.math.MathUtil;

/**
 * Utility methods for motor calculations.
 */
public class MotorUtils {
    /**
     * Calculates the target position for angle control with wrap-around.
     * 
     * <p>Determines the shortest path to reach the target angle, accounting for
     * wrap-around at ±180° or ±π. This prevents unnecessary full rotations.</p>
     * 
     * <p><b>Example:</b></p>
     * <pre>
     * // Current position: 350°, target angle: 10°
     * // Returns a position that results in 20° forward rotation, not 340° backward
     * double targetPos = getPositionForAngle(currentPos, 10, false);
     * </pre>
     * 
     * @param position Current encoder position
     * @param angle Desired target angle
     * @param isRadians true if using radians, false if using degrees
     * @return Target position that takes shortest path to angle
     */
    public static double getPositionForAngle(double position, double angle, boolean isRadians) {
        double pi = isRadians ? Math.PI : 180;
        double currentAngle = MathUtil.inputModulus(position, -pi, pi);
        double diff = angle - currentAngle;
        if (diff > pi) {
            diff = pi * 2 - diff;
        } else if (diff < -pi) {
            diff = pi * 2 + diff;
        }
        return position + diff;
    }

}
