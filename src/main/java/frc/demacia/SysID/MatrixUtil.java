package frc.Demacia.Sysid;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;

public class MatrixUtil {

    @SuppressWarnings({ "rawtypes", "unchecked" })
      public static Matrix exp(Matrix A) {
          // calculate matrix exponential
          // using taylor terms
          // exp(A) = I + A + A²/2! + A³/3! + ... + Aⁿ/n!
          // using 20 series
          var res = new Matrix<>(SimpleMatrix.identity(A.getNumCols())).plus(A); // I + A
          var t = A.copy();
          double factorial = 1;
          for(double i = 2; i < 20; i++) {
              factorial *= i;
              t = t.times(A).times(1/factorial);
              res = res.plus(t);
          }
          return res;
      }

}
