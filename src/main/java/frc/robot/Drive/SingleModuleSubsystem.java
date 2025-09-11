package frc.robot.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleModuleSubsystem extends SubsystemBase {

    SwerveModule[] modules;
    SwerveModulePosition[] modulePositions;
    SwerveModuleState[] moduleState;

    public SingleModuleSubsystem() {
        super();
        modules = new SwerveModule[Constants.CONFIGS1.length];
        Translation2d[] modulePositionOnRobot = new Translation2d[modules.length];
        modulePositions = new SwerveModulePosition[modules.length];
        moduleState = new SwerveModuleState[modules.length];
        for(int i = 0; i < modules.length; i++) {
            modules[i] = new SwerveModule(Constants.CONFIGS1[i]);
            modulePositionOnRobot[i] = modules[i].config.positionRelativeToRobotCenter;
            moduleState[i] = modules[i].state;
            modulePositions[i] = modules[i].position;
            modules[i].showConfigPID();
            modules[i].showBaseCommands(this);
        }
        SmartDashboard.putData("SingleModule", this);
    }

    @Override
    public void periodic() {
        super.periodic();
        for(SwerveModule m : modules) {
            m.refreshStateAndPosition();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("state angle",()->moduleState[0].angle.getDegrees(), null);
        builder.addDoubleProperty("state velocity",()->moduleState[0].speedMetersPerSecond, null);
        builder.addDoubleProperty("position angle",()->modulePositions[0].angle.getDegrees(), null);
        builder.addDoubleProperty("position distance",()->modulePositions[0].distanceMeters, null);
    }

}
