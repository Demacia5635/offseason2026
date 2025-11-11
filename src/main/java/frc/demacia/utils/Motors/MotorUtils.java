package frc.demacia.utils.Motors;

import edu.wpi.first.math.MathUtil;

public class MotorUtils {
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
