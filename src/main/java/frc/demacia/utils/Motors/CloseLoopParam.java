package frc.Demacia.utils.Motors;

/** 
* Class to hold closed loop param
*  */
class CloseLoopParam { // calculate volts - not -1 to 1 !!!

    public static String[] names= {"kp", "ki", "kd", "ks", "kv", "ka", "kg"};

    private double[] array = {0,0,0,0,0,0,0};


    CloseLoopParam() {
    }

    CloseLoopParam(double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        set(kp,ki,kd,ks,kv,ka,kg);
    }
    CloseLoopParam(double kp, double ki, double kd, double kf) {
        set(kp,ki,kd,0,kf,0,0);
    }

    public void set (double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        array[0] = kp;
        array[1] = ki;
        array[2] = kd;
        array[3] = ks;
        array[4] = kv;
        array[5] = ka;
        array[6] = kg;
    }

    public void set(CloseLoopParam other) {
        array = other.array.clone();
    }

    public double[] toArray() {
        return array;
    }
    public double kp() {
        return array[0];
    }

    public double ki() {
        return array[1];
    }

    public double kd() {
        return array[2];
    }

    public double ks() {
        return array[3];
    }

    public double kv() {
        return array[4];
    }

    public double ka() {
        return array[5];
    }

    public double kg() {
        return array[6];
    }
}