package frc.demacia.utils.Motors;

/**
 * Container class for closed-loop control parameters (PID + feed-forward).
 * 
 * <p>Stores seven control parameters used for precise motor control:</p>
 * <ul>
 *   <li>kp, ki, kd - PID gains</li>
 *   <li>ks - Static friction compensation</li>
 *   <li>kv - Velocity feed-forward</li>
 *   <li>ka - Acceleration feed-forward</li>
 *   <li>kg - Gravity feed-forward</li>
 * </ul>
 * 
 * <p><b>Note:</b> This class calculates output in <i>volts</i>, not normalized [-1, 1].</p>
 */
class CloseLoopParam {

    public static String[] PARAMETER_NAMES = {"kp", "ki", "kd", "ks", "kv", "ka", "kg"};

    private double[] parameters = {0,0,0,0,0,0,0};

    /**
     * Default constructor. Initializes all parameters to zero.
     */
    CloseLoopParam() {}

    /**
     * Constructor with all seven control parameters.
     * 
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param ks Static friction feed-forward (volts)
     * @param kv Velocity feed-forward (volts per unit/sec)
     * @param ka Acceleration feed-forward (volts per unit/sec²)
     * @param kg Gravity feed-forward (volts)
     */
    CloseLoopParam(double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        set(kp,ki,kd,ks,kv,ka,kg);
    }

    /**
     * Simplified constructor with feed-forward (legacy).
     * 
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param kf Feed-forward gain (mapped to kv)
     */
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