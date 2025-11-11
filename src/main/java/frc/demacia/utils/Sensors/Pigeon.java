package frc.demacia.utils.Sensors;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.demacia.utils.UpdateArray;
import frc.demacia.utils.Log.LogManager;

import com.ctre.phoenix6.StatusSignal;

public class Pigeon extends Pigeon2 implements SensorInterface{
    PigeonConfig config;
    String name;
    Pigeon2Configuration pigeonConfig;

    StatusSignal<Angle> yawSignal;
    StatusSignal<Angle> pitchSignal;
    StatusSignal<Angle> rollSignal;
    StatusSignal<AngularVelocity> xVelocitySignal;
    StatusSignal<AngularVelocity> yVelocitySignal;
    StatusSignal<AngularVelocity> zVelocitySignal;
    StatusSignal<LinearAcceleration> xAccelerationSignal;
    StatusSignal<LinearAcceleration> yAccelerationSignal;
    StatusSignal<LinearAcceleration> zAccelerationSignal;

    double lastYaw;
    double lastPitch;
    double lastRoll;
    double lastXVelocity;
    double lastYVelocity;
    double lastZVelocity;
    double lastXAcceleration;
    double lastYAcceleration;
    double lastZAcceleration;

    public Pigeon(PigeonConfig config){
        super(config.id, config.canbus);
        this.config = config;
        name = config.name;
        configPigeon();
        setStatusSignals();
        addLog();
		LogManager.log(name + " pigeon initialized");
    }
    private void configPigeon() {
        pigeonConfig = new Pigeon2Configuration();
        pigeonConfig.MountPose.MountPosePitch = config.pitchOffset;
        pigeonConfig.MountPose.MountPoseRoll = config.rollOffset;
        pigeonConfig.MountPose.MountPoseYaw = config.yawOffset;
        pigeonConfig.GyroTrim.GyroScalarX = config.isInverted?-config.xScalar:config.xScalar;
        pigeonConfig.GyroTrim.GyroScalarY = config.isInverted?-config.yScalar:config.yScalar;
        pigeonConfig.GyroTrim.GyroScalarZ = config.isInverted?-config.zScalar:config.zScalar;
        pigeonConfig.Pigeon2Features.EnableCompass = config.compass;
        pigeonConfig.Pigeon2Features.DisableTemperatureCompensation = !config.temperatureCompensation;
        pigeonConfig.Pigeon2Features.DisableNoMotionCalibration = !config.noMotionCalibration;
        getConfigurator().apply(pigeonConfig);
    }

    private void setStatusSignals(){
        yawSignal = getYaw();
        pitchSignal = getPitch();
        rollSignal = getRoll();
        xVelocitySignal = getAngularVelocityXWorld();
        yVelocitySignal = getAngularVelocityYWorld();
        zVelocitySignal = getAngularVelocityZWorld();
        xAccelerationSignal = getAccelerationX();
        yAccelerationSignal = getAccelerationY();
        zAccelerationSignal = getAccelerationZ();

        lastYaw = yawSignal.getValueAsDouble();
        lastPitch = pitchSignal.getValueAsDouble();
        lastRoll = rollSignal.getValueAsDouble();
        lastXVelocity = xVelocitySignal.getValueAsDouble();
        lastYVelocity = yVelocitySignal.getValueAsDouble();
        lastZVelocity = zVelocitySignal.getValueAsDouble();
        lastXAcceleration = xAccelerationSignal.getValueAsDouble();
        lastYAcceleration = yAccelerationSignal.getValueAsDouble();
        lastZAcceleration = zAccelerationSignal.getValueAsDouble();
    }

    public void checkElectronics() {
        if (getFaultField().getValue() != 0) {
            LogManager.log(name + " have a fault: " + getFaultField().getValue());
        }
    }

    private void addLog() {
        LogManager.addEntry(name + "/yaw and pitch and x velocity and y velocity and z velocity", () -> new double[] {
            getCurrentYaw(),
            getCurrentPitch(),
            getXVelocity(),
            getYVelocity(),
            getZVelocity(),
            getXAcceleration(),
            getYAcceleration(),
            getZAcceleration(),
            getXAngularAcceleration(),
            getYAngularAcceleration(),
            getZAngularAcceleration()
        }, 3);
        LogManager.addEntry(name + "/yaw", () -> getYaw() , 3);
    }

    public String getName(){
        return config.name;
    }

    public double getCurrentYaw() {
        lastYaw = StatusSignalHelper.getStatusSignalInRad(yawSignal, lastYaw);
        return lastYaw;
    }

    public double getYawInZeroTo2Pi() {
        return (getCurrentYaw()% (2* Math.PI) + (2* Math.PI)) % (2* Math.PI);
    }

    public double getCurrentPitch() {
        lastPitch = StatusSignalHelper.getStatusSignalInRad(pitchSignal, lastPitch);
        return lastPitch;
    }

    public double getPitchInZeroTo2Pi() {
        return (getCurrentPitch() % (2* Math.PI) + (2* Math.PI)) % (2* Math.PI);
    }

    public double getCurrentRoll() {
        lastRoll = StatusSignalHelper.getStatusSignalInRad(rollSignal, lastRoll);
        return lastRoll;
    }

    public double getRollInZeroTo2Pi() {
        return (getCurrentRoll()% (2* Math.PI) + (2* Math.PI)) % (2* Math.PI);
    }

    public double getXVelocity() {
        lastXVelocity = StatusSignalHelper.getStatusSignalBasic(xVelocitySignal, lastXVelocity);
        return lastXVelocity;
    }

    public double getYVelocity() {
        lastYVelocity = StatusSignalHelper.getStatusSignalBasic(yVelocitySignal, lastYVelocity);
        return lastYVelocity;
    }

    public double getZVelocity() {
        lastZVelocity = StatusSignalHelper.getStatusSignalBasic(zVelocitySignal, lastZVelocity);
        return lastZVelocity;
    }

    public double getXAcceleration() {
        lastXAcceleration = StatusSignalHelper.getStatusSignalBasic(xAccelerationSignal, lastXAcceleration);
        return lastXAcceleration;
    }

    public double getYAcceleration() {
        lastYAcceleration = StatusSignalHelper.getStatusSignalBasic(yAccelerationSignal, lastYAcceleration);
        return lastYAcceleration;
    }

    public double getZAcceleration() {
        lastZAcceleration = StatusSignalHelper.getStatusSignalBasic(zAccelerationSignal, lastZAcceleration);
        return lastZAcceleration;
    }

    public double getXAngularAcceleration() {
        double acceleration = (StatusSignalHelper.getStatusSignalBasic(xVelocitySignal, lastXVelocity)) - lastXVelocity;
        lastXVelocity = xVelocitySignal.getValueAsDouble();
        return acceleration;
    }

    public double getYAngularAcceleration() {
        double acceleration = (StatusSignalHelper.getStatusSignalBasic(yVelocitySignal, lastYVelocity)) - lastYVelocity;
        lastYVelocity = yVelocitySignal.getValueAsDouble();
        return acceleration;
    }

    public double getZAngularAcceleration() {
        double acceleration = (StatusSignalHelper.getStatusSignalBasic(zVelocitySignal, lastZVelocity)) - lastZVelocity;
        lastZVelocity = zVelocitySignal.getValueAsDouble();
        return acceleration;
    }

    public void reset() {
        super.reset();
    }

public void showConfigMotorCommand() {
        UpdateArray.show(name + " CONFIG",
            new String[] {
                "pitch Offset",
                "roll Offset",
                "yaw Offset",
                "x Scalar",
                "y Scalar",
                "z Scalar",
                "is Inverted (1, 0)"
            }, 
            new double[] {
                config.pitchOffset,
                config.rollOffset,
                config.yawOffset,
                config.xScalar,
                config.yScalar,
                config.zScalar,
                config.isInverted ? 1.0 : 0.0
            },
            (double[] array) -> {
                config.withPitchOffset(array[0])
                .withRollOffset(array[1])
                .withYawOffset(array[2])
                .withXScalar(array[3])
                .withYScalar(array[4])
                .withZScalar(array[5])
                .withInvert(array[6] > 0.5);
                
                configPigeon();
            }
        );
    }
}