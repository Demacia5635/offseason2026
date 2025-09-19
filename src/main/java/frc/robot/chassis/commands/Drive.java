package frc.robot.chassis.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.chassis.utils.ChassisConstants;
import frc.robot.RobotContainer;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.CommandController;

public class Drive extends Command {
    private Chassis chassis;
    private CommandController controller;
    private double direction;
    private boolean isRed;
    private ChassisSpeeds speeds;
    private static boolean precisionMode;

    public Drive(Chassis chassis, CommandController controller) {
        this.chassis = chassis;
        this.controller = controller;
        Drive.precisionMode = false;

        addRequirements(chassis);
    }

    public static void invertPrecisionMode() {
        setPrecisionMode(!precisionMode);
    }
    public static void setPrecisionMode(boolean precisionMode) {
        Drive.precisionMode = precisionMode;
    }
    public static boolean getPrecisionMode() {
        return precisionMode;
    }

    @Override
    public void execute() {
        isRed = chassis.isRed();
        direction = isRed ? 1 : -1;
        double joyX = controller.getLeftY() * direction;
        double joyY = controller.getLeftX() * direction;
        
        // Calculate r]otation from trigger axes
        double rot = controller.getLeftTrigger() - controller.getRightTrigger();
        
        double velX = Math.pow(joyX, 2) * ChassisConstants.MAX_DRIVE_VELOCITY * Math.signum(joyX);
        double velY = Math.pow(joyY, 2) * ChassisConstants.MAX_DRIVE_VELOCITY * Math.signum(joyY);
        double velRot = Math.pow(rot, 2) * ChassisConstants.MAX_ROTATIONAL_VELOCITY * Math.signum(rot);
        if(precisionMode){
            velX /= 4;
            velY /= 4;
            velRot /= 4;
        }
        
        speeds = new ChassisSpeeds(velX, velY,velRot);
 
        chassis.setVelocities(speeds);
    }
}
