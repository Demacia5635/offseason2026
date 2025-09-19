package frc.robot.chassis.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.chassis.utils.ChassisConstants.SwerveModuleConfigs;
import frc.robot.utils.Cancoder;
import frc.robot.utils.TalonMotor;

public class SwerveModule {
    private TalonMotor steerMotor;
    private TalonMotor driveMotor;
    private Cancoder cancoder;
    public String name;

    public SwerveModule(SwerveModuleConfigs configs) {
        steerMotor = new TalonMotor(configs.STEER_CONFIG);
        driveMotor = new TalonMotor(configs.DRIVE_CONFIG);
        cancoder = new Cancoder(configs.CANCODER_CONFIG);
        name = configs.NAME;

        steerMotor.setPosition(getAbsoluteAngle() - configs.STEER_OFFSET);
        
        // SmartDashboard.putData(configs.DRIVE_CONFIG.name, driveMotor);
        // SmartDashboard.putData(configs.STEER_CONFIG.name, steerMotor);
    }

    public void checkElectronics() {
        driveMotor.checkElectronics();
        steerMotor.checkElectronics();
        cancoder.checkElectronics();
    }

    public void setNeutralMode(boolean isBrake) {
        driveMotor.setNeutralMode(isBrake);
        steerMotor.setNeutralMode(isBrake);
    }

    public void setSteerPower(double power) {
        steerMotor.set(power);
    }

    public double getAbsoluteAngle() {
        return cancoder.getCurrentAbsPosition();
    }

    public void setDrivePower(double power) {
        driveMotor.set(power);
    }

    public void setSteerVelocity(double velocityRadsPerSecond) {
        steerMotor.setVelocity(velocityRadsPerSecond);
    }

    public void setDriveVelocity(double velocityMetersPerSecond) {
        driveMotor.setVelocity(velocityMetersPerSecond);
    }

    public void setSteerPosition(double positionRadians) {
        steerMotor.setPositionVoltage(positionRadians);
        // steerMotor.setMotionMagic(positionRadians);
    }

    public double getSteerAngle() {
        return steerMotor.getCurrentPosition();
    }
    public Rotation2d getSteerRotation() {
        return new Rotation2d(getSteerAngle());
    }
    public double getSteerVel() {
        return steerMotor.getCurrentVelocity();
    }
    public double getSteerAccel() {
        return steerMotor.getAcceleration().getValueAsDouble();
    }

    public double getDriveVel() {
        return driveMotor.getCurrentVelocity();
    }

    public void setState(SwerveModuleState state) {
        double wantedAngle = state.angle.getRadians();
        double diff = wantedAngle - steerMotor.getCurrentPosition();
        double vel = state.speedMetersPerSecond;
        diff = MathUtil.angleModulus(diff);
        if(diff > 0.5 * Math.PI) {
            vel = -vel;
            diff = diff-Math.PI;
        } else if(diff < -0.5 * Math.PI) {
            vel = -vel;
            diff = diff + Math.PI;
        }

        setSteerPosition(steerMotor.getCurrentPosition() + diff);
        setDriveVelocity(vel);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveMotor.getCurrentPosition(), Rotation2d.fromRadians(steerMotor.getCurrentPosition()));
    }

    /**
     * Returns the state of the module
     * @return Velocity in m/s
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVel(), getSteerRotation());
    }


    public void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }
}
