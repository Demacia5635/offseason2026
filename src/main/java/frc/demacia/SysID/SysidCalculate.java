package frc.Demacia.Sysid;

import java.util.ArrayList;
import java.util.EnumSet;

import org.ejml.simple.SimpleMatrix;

import frc.Demacia.Sysid.MotorData.MotorTimeData;

public class SysidCalculate {

    public static enum VelocityRange { SLOW, MID, HIGH};
    public static enum KTypes { KS, KV, KA, KGElevator, KGArm, KV2, KSqrt};

    double[] vRange;
    ArrayList<ArrayList<MotorTimeData>> data = new ArrayList<>();
    double[] maxError = {0,0,0};
    double[] avgError = {0,0,0};
    double[][] k = new double[KTypes.values().length][VelocityRange.values().length];
    double[] kp = {0,0,0};
    
    MotorData motorData;
    EnumSet<KTypes> kTypes;

    public SysidCalculate(MotorData motor,EnumSet<KTypes> types) {
        this.motorData = motor;
        this.kTypes = types;
        kTypes.add(KTypes.KS);
        kTypes.add(KTypes.KV);
        kTypes.add(KTypes.KA);
        calculate();
    }

    private boolean valid(double value, double min) {
        return value > min || value < -min;
    }

    // return the array to use - -1 if filterred
    private int range(MotorTimeData data) {
        double v = Math.abs(data.velocity);
        int i = v < vRange[0] ? 0 : v < vRange[1] ? 1 : 2;
        
        if(valid(v, 0.1) && valid(data.voltage, 0.05)) {
            if(data.prev != null) {
                if(valid(data.prev.velocity, 0.1) && valid(data.prev.voltage, 0.2)) {
                    return i;
                } else {
                    return -1;
                }
            } else {
                return i;
            }
        } else {
            return -1;
        }
    }

    private double getValue(MotorTimeData data, KTypes type) {
        switch (type) {
            case KS:
                return Math.signum(data.velocity);
            case KV:
                return data.velocity;
            case KA:
                return data.acceleration;
            case KGElevator:
                return 1;
            case KGArm:
                return Math.cos(data.position);
            case KV2:
                return Math.signum(data.velocity)*data.velocity*data.velocity;
            case KSqrt:
                return Math.signum(data.velocity)*Math.sqrt(Math.abs(data.velocity));
            default:
                return 0;
        }
    }

    private void calculate() {
        double maxV = motorData.maxVelocity();
        vRange = new double[] { maxV*0.3, maxV*0.7, maxV};
        for(int i = 0; i < 3; i++) {
            data.add(new ArrayList<>());
        }
        // fill the array of data - filterring data based on minimum voltage and velocity
        for(MotorTimeData d : motorData.data()) {
            int range = range(d);
            if(range >= 0) {
                data.get(range).add(d);
            }
        }
        int col = kTypes.size();
        for(VelocityRange range: VelocityRange.values()) { // for each range
            int i = range.ordinal();
            ArrayList<MotorTimeData> dataArray = data.get(i);
            if(dataArray.size() > 50) { // only if have enough data
                // fill the data and volts matrix
                SimpleMatrix mat = new SimpleMatrix(dataArray.size(), col);
                SimpleMatrix volt = new SimpleMatrix(dataArray.size(), 1);
                for(int row  = 0; row < dataArray.size(); row++) {
                    MotorTimeData d = dataArray.get(row);
                    int c = 0;
                    for(KTypes kType: kTypes) {
                        mat.set(row, c++, getValue(d, kType));
                    }
                    volt.set(row, 0, d.voltage);
                }
                // solve
                SimpleMatrix result = mat.solve(volt);
                // calculate error
                SimpleMatrix predict = mat.mult(result);
                SimpleMatrix error = volt.minus(predict);
                double maxErr = 0;
                double sumError = 0;
                for(int e = 0; e < error.getNumRows(); e++) {
                    MotorTimeData md = dataArray.get(e);
                    double val = Math.abs(error.get(e, 0) / md.voltage);
                    if(val > maxErr)
                        maxErr = val;
                    sumError += val;
                }
                double avgErr = sumError / dataArray.size();
                // update result
                int c = 0;
                for(KTypes kType: kTypes) {
                    k[kType.ordinal()][i] = result.get(c++,0);                    
                }
                maxError[i] = maxErr;
                avgError[i] = avgErr;
                if( k[KTypes.KA.ordinal()][i] > 0.0001) {
                    kp[i] = CalculateFeedbackGains.calculateFeedbackGains(k[KTypes.KV.ordinal()][i], k[KTypes.KA.ordinal()][i]);
                } else {
                    kp[i] = 0;
                }
                // print the worst cases
                for(int e = 0; e < error.getNumRows(); e++) {
                    MotorTimeData md = dataArray.get(e);
                    double val = Math.abs(error.get(e, 0) / md.voltage);
                    if(val > 0.5 && val > avgErr * 4) {
                        Sysid.msg(String.format("error %4.2f%% for %s", val, md.toString()));
                    }
                }
            }
        }
        Sysid.msg(String.format("Min Power to Move = %.3f",motorData.minPowerToMove));
    }

    public double getRange(VelocityRange range) {
        return vRange[range.ordinal()];
    }
    public int getCount(VelocityRange range) {
        return data.get(range.ordinal()).size();
    }
    public double getMaxError(VelocityRange range) {
        return maxError[range.ordinal()];
    }
    public double getAverageError(VelocityRange range) {
        return avgError[range.ordinal()];
    }
    public double getK(KTypes type, VelocityRange range) {
        return k[type.ordinal()][range.ordinal()];
    }
    public double getKP(VelocityRange range) {
        return kp[range.ordinal()];
    }
}
