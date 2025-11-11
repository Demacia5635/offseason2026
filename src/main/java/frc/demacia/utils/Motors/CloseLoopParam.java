package frc.demacia.utils.Motors;

/** 
* Class to hold closed loop param
*  */
class CloseLoopParam { // calculate volts - not -1 to 1 !!!

    public static String[] PARAMETER_NAMES = {"kp", "ki", "kd", "ks", "kv", "ka", "kg"};

    private double[] parameters = {0,0,0,0,0,0,0};


    CloseLoopParam() {
    }

    CloseLoopParam(double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        set(kp,ki,kd,ks,kv,ka,kg);
    }
    CloseLoopParam(double kp, double ki, double kd, double kf) {
        set(kp,ki,kd,0,kf,0,0);
    }

    public void set (double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        parameters[0] = kp;
        parameters[1] = ki;
        parameters[2] = kd;
        parameters[3] = ks;
        parameters[4] = kv;
        parameters[5] = ka;
        parameters[6] = kg;
    }

    public void set(CloseLoopParam other) {
        parameters = other.parameters.clone();
    }

    public double[] toArray() {
        return parameters;
    }
    public double kp() {
        return parameters[0];
    }

    public double ki() {
        return parameters[1];
    }

    public double kd() {
        return parameters[2];
    }

    public double ks() {
        return parameters[3];
    }

    public double kv() {
        return parameters[4];
    }

    public double ka() {
        return parameters[5];
    }

    public double kg() {
        return parameters[6];
    }
}