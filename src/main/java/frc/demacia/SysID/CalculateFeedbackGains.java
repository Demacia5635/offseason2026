package frc.Demacia.Sysid;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class CalculateFeedbackGains {
    public static double calculateFeedbackGains(double kv, double ka) {
        var plant = LinearSystemId.identifyVelocitySystem(kv, ka);
        var A = plant.getA();
        var B = plant.getB();
        var Q = StateSpaceUtil.makeCostMatrix(VecBuilder.fill(0.1));
        var R = StateSpaceUtil.makeCostMatrix(VecBuilder.fill(12));
        var discABPair = Discretization.discretizeAB(A, B, 0.02);
        var discA = discABPair.getFirst();
        var discB = discABPair.getSecond();

        var S = new Matrix<N1, N1>(RiccatiSolver.solveDARE(discA.getStorage(), discB.getStorage(), Q.getStorage(), R.getStorage()));

        // K = (BᵀSB + R)⁻¹BᵀSA
        var K = discB
            .transpose()
            .times(S)
            .times(discB)
            .plus(R)
            .solve(discB.transpose().times(S).times(discA));

        double kp = K.get(0, 0);
        return kp;
    }
    
    public static void main(String[] args) {
        try {
            double kp = calculateFeedbackGains(0.5, 0.5);
            System.out.println("kp = " + kp);
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

}
