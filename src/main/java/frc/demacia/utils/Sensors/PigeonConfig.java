package frc.demacia.utils.Sensors;

import com.ctre.phoenix6.CANBus;

public class PigeonConfig extends BaseSensorConfig<PigeonConfig>{
    public double pitchOffset = 0;
    public double rollOffset = 0;
    public double yawOffset = 0;
    public double xScalar = 1;
    public double yScalar = 1;
    public double zScalar = 1;
    public boolean compass = false;
    public boolean temperatureCompensation = false;
    public boolean noMotionCalibration = false;
    /** 
     * Constructor
     * @param id - canbus ID1
     * @param canbus - Name of canbus
     * @param name - name of Pigeon for logging
     */
    public PigeonConfig(int id, CANBus canbus, String name) {
        super(id, canbus, name);
        sensorType = Pigeon.class;
    }


    public PigeonConfig withPitchOffset(double pitchOffset) {
        this.pitchOffset = pitchOffset;
        return this;
    }


    public PigeonConfig withRollOffset(double rollOffset) {
        this.rollOffset = rollOffset;
        return this;
    }


    public PigeonConfig withYawOffset(double yawOffset) {
        this.yawOffset = yawOffset;
        return this;
    }


    public PigeonConfig withXScalar(double xScalar) {
        this.xScalar = xScalar;
        return this;
    }


    public PigeonConfig withYScalar(double yScalar) {
        this.yScalar = yScalar;
        return this;
    }


    public PigeonConfig withZScalar(double zScalar) {
        this.zScalar = zScalar;
        return this;
    }


    public PigeonConfig withCompass(boolean compass) {
        this.compass = compass;
        return this;
    }


    public PigeonConfig withTemperatureCompensation(boolean temperatureCompensation) {
        this.temperatureCompensation = temperatureCompensation;
        return this;
    }


    public PigeonConfig withNoMotionCalibration(boolean noMotionCalibration) {
        this.noMotionCalibration = noMotionCalibration;
        return this;
    }
}
