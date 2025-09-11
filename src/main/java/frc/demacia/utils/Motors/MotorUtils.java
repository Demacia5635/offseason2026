package frc.Demacia.utils.Motors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MotorUtils {

    public static void showSysidCommands(MotorInterface motor, @SuppressWarnings("rawtypes") BaseMotorConfig config,
            Subsystem subsystem) {
        motor.showConfigMotionVelocitiesCommand();
        motor.showConfigPIDFSlotCommand(0);
        MotorCommands.showRandomPowerCommand(motor.name() + " Random Power", config.minVolt, config.maxVolt,
                Math.max(0.2, config.rampUpTime), subsystem, motor);
        if (config.isMeterMotor) {
            MotorCommands.showVelocityCommand(motor.name() + " Velocity Command", subsystem, motor);
        } else {
            MotorCommands.showSlowPowerCommand(motor.name() + " Slow Power", 0.0, 0.01, 1, subsystem, motor);
            MotorCommands.showAngleCommand(motor.name() + " Angle Command", subsystem, motor);
        }
    }

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
