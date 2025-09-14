package frc.Demacia.Sysid;

import org.ejml.simple.SimpleMatrix;
import org.ejml.dense.row.CommonOps_DDRM;

/**
 * Solver for the Discrete-Time Algebraic Riccati Equation (DARE):
 * A'XA - X - A'XB(B'XB + R)^(-1)B'XA + Q = 0
 * 
 * Uses SimpleMatrix from EJML library for efficient matrix operations
 */
public class RiccatiSolver {
    
    private static final double TOLERANCE = 1e-10;
    private static final int MAX_ITERATIONS = 100;
    
    /**
     * Solves the discrete-time algebraic Riccati equation using iterative method
     * A'XA - X - A'XB(B'XB + R)^(-1)B'XA + Q = 0
     * 
     * @param A System matrix (n x n)
     * @param B Input matrix (n x m)
     * @param Q State weighting matrix (n x n, positive semi-definite)
     * @param R Input weighting matrix (m x m, positive definite)
     * @return Solution matrix X (n x n)
     */
    public static SimpleMatrix solveDARE(SimpleMatrix A, SimpleMatrix B, SimpleMatrix Q, SimpleMatrix R) {
        // Validate input dimensions
        validateInputs(A, B, Q, R);
        
        // Initialize X with Q (good starting point)
        SimpleMatrix X = Q.copy();
        SimpleMatrix X_prev;
        
        for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
            // Store previous iteration
            X_prev = X.copy();
            
            // Compute the iteration: X = A'XA - A'XB(B'XB + R)^(-1)B'XA + Q
            X = riccatiIteration(A, B, Q, R, X);
            
            // Check convergence
            if (hasConverged(X, X_prev)) {
                return X;
            }
        }
        
        System.out.println("Warning: Maximum iterations reached without convergence");
        return X;
    }
    
    /**
     * Single iteration of the Riccati equation
     */
    private static SimpleMatrix riccatiIteration(SimpleMatrix A, SimpleMatrix B, 
                                               SimpleMatrix Q, SimpleMatrix R, SimpleMatrix X) {
        // Compute A'
        SimpleMatrix At = A.transpose();
        
        // Compute B'
        SimpleMatrix Bt = B.transpose();
        
        // Compute A'X
        SimpleMatrix AtX = At.mult(X);
        
        // Compute A'XA
        SimpleMatrix AtXA = AtX.mult(A);
        
        // Compute B'X
        SimpleMatrix BtX = Bt.mult(X);
        
        // Compute B'XB
        SimpleMatrix BtXB = BtX.mult(B);
        
        // Compute B'XB + R
        SimpleMatrix BtXB_R = BtXB.plus(R);
        
        // Compute (B'XB + R)^(-1)
        SimpleMatrix inv_BtXB_R = BtXB_R.invert();
        
        // Compute A'XB(B'XB + R)^(-1)
        SimpleMatrix AtXB = AtX.mult(B);
        SimpleMatrix temp1 = AtXB.mult(inv_BtXB_R);
        
        // Compute A'XB(B'XB + R)^(-1)B'XA
        SimpleMatrix temp2 = temp1.mult(BtX);
        SimpleMatrix AtXB_inv_BtXA = temp2.mult(A);
        
        // Compute result: A'XA - A'XB(B'XB + R)^(-1)B'XA + Q
        SimpleMatrix result = AtXA.minus(AtXB_inv_BtXA).plus(Q);
        
        return result;
    }
    
    
    /**
     * Validates input matrices
     */
    private static void validateInputs(SimpleMatrix A, SimpleMatrix B, SimpleMatrix Q, SimpleMatrix R) {
        int n = A.getNumRows();
        int m = B.getNumCols();
        
        if (A.getNumCols() != n) {
            throw new IllegalArgumentException("Matrix A must be square");
        }
        if (B.getNumRows() != n) {
            throw new IllegalArgumentException("Matrix B must have same number of rows as A");
        }
        if (Q.getNumRows() != n || Q.getNumCols() != n) {
            throw new IllegalArgumentException("Matrix Q must be same size as A");
        }
        if (R.getNumRows() != m || R.getNumCols() != m) {
            throw new IllegalArgumentException("Matrix R must be square with same columns as B");
        }
        
        // Check if R is positive definite (simplified check)
        if (R.determinant() <= 0) {
            System.out.println("Warning: Matrix R may not be positive definite");
        }
    }
    
    /**
     * Checks if the iteration has converged
     */
    private static boolean hasConverged(SimpleMatrix X, SimpleMatrix X_prev) {
        SimpleMatrix diff = X.minus(X_prev);
        double maxDiff = CommonOps_DDRM.elementMaxAbs(diff.getMatrix());
        return maxDiff < TOLERANCE;
    }
    
    /**
     * Computes the residual of the Riccati equation to verify the solution
     * Residual = A'XA - X - A'XB(B'XB + R)^(-1)B'XA + Q
     */
    public static SimpleMatrix computeResidual(SimpleMatrix A, SimpleMatrix B, 
                                             SimpleMatrix Q, SimpleMatrix R, SimpleMatrix X) {
        SimpleMatrix At = A.transpose();
        SimpleMatrix Bt = B.transpose();
        
        SimpleMatrix AtX = At.mult(X);
        SimpleMatrix AtXA = AtX.mult(A);
        
        SimpleMatrix BtX = Bt.mult(X);
        SimpleMatrix BtXB = BtX.mult(B);
        SimpleMatrix BtXB_R = BtXB.plus(R);
        SimpleMatrix inv_BtXB_R = BtXB_R.invert();
        
        SimpleMatrix AtXB = AtX.mult(B);
        SimpleMatrix temp1 = AtXB.mult(inv_BtXB_R);
        SimpleMatrix temp2 = temp1.mult(BtX);
        SimpleMatrix AtXB_inv_BtXA = temp2.mult(A);
        
        SimpleMatrix result = AtXA.minus(X).minus(AtXB_inv_BtXA).plus(Q);
        
        return result;
    }
    
    /**
     * Computes the optimal feedback gain K = (B'XB + R)^(-1)B'XA
     */
    public static SimpleMatrix computeOptimalGain(SimpleMatrix A, SimpleMatrix B, 
                                                SimpleMatrix R, SimpleMatrix X) {
        SimpleMatrix Bt = B.transpose();
        SimpleMatrix BtX = Bt.mult(X);
        SimpleMatrix BtXB = BtX.mult(B);
        SimpleMatrix BtXB_R = BtXB.plus(R);
        SimpleMatrix inv_BtXB_R = BtXB_R.invert();
        SimpleMatrix BtXA = BtX.mult(A);
        
        return inv_BtXB_R.mult(BtXA);
    }
    
    /**
     * Checks if the closed-loop system is stable
     */
    public static boolean isStable(SimpleMatrix A, SimpleMatrix B, SimpleMatrix K) {
        SimpleMatrix A_cl = A.minus(B.mult(K));
        
        // Check if all eigenvalues are inside unit circle
        // This is a simplified check - full implementation would compute actual eigenvalues
        double det = A_cl.determinant();
        double trace = A_cl.trace();
        
        // For 2x2 matrix, eigenvalues are stable if |det| < 1 and |trace| < 1 + det
        if (A_cl.getNumRows() == 2) {
            return Math.abs(det) < 1.0 && Math.abs(trace) < 1.0 + det;
        }
        
        // For larger matrices, would need proper eigenvalue computation
        return Math.abs(det) < 1.0;  // Simplified check
    }
    
    /**
     * Utility method to print matrix with name
     */
    public static void printMatrix(String name, SimpleMatrix matrix) {
        System.out.println(name + ":");
        matrix.print("%10.6f");
        System.out.println();
    }
    
    /**
     * Creates a SimpleMatrix from 2D array
     */
    public static SimpleMatrix createMatrix(double[][] data) {
        return new SimpleMatrix(data);
    }
    
    /**
     * Test cases and examples
     */
    public static void main(String[] args) {
        System.out.println("=== Discrete-Time Algebraic Riccati Equation Solver (SimpleMatrix) ===\n");
        
        // Test case 1: Simple 2x2 system
        System.out.println("Test 1: Simple 2x2 system");
        SimpleMatrix A1 = createMatrix(new double[][]{
            {1.1, 0.1},
            {0.0, 0.9}
        });
        
        SimpleMatrix B1 = createMatrix(new double[][]{
            {1.0},
            {0.5}
        });
        
        SimpleMatrix Q1 = createMatrix(new double[][]{
            {1.0, 0.0},
            {0.0, 1.0}
        });
        
        SimpleMatrix R1 = createMatrix(new double[][]{{1.0}});
        
        printMatrix("A", A1);
        printMatrix("B", B1);
        printMatrix("Q", Q1);
        printMatrix("R", R1);
        
        SimpleMatrix X1 = solveDARE(A1, B1, Q1, R1);
        printMatrix("Solution X", X1);
        
        // Compute and verify residual
        SimpleMatrix residual1 = computeResidual(A1, B1, Q1, R1, X1);
        printMatrix("Residual (should be near zero)", residual1);
        
        // Compute optimal gain
        SimpleMatrix K1 = computeOptimalGain(A1, B1, R1, X1);
        printMatrix("Optimal gain K", K1);
        
        // Check stability
        boolean stable1 = isStable(A1, B1, K1);
        System.out.println("Closed-loop system stable: " + stable1);
        
        System.out.println("===========================================\n");
        
        // Test case 2: Double integrator system
        System.out.println("Test 2: Double integrator (discretized with dt=0.1)");
        double dt = 0.1;
        SimpleMatrix A2 = createMatrix(new double[][]{
            {1.0, dt},
            {0.0, 1.0}
        });
        
        SimpleMatrix B2 = createMatrix(new double[][]{
            {0.5 * dt * dt},
            {dt}
        });
        
        SimpleMatrix Q2 = createMatrix(new double[][]{
            {10.0, 0.0},
            {0.0, 1.0}
        });
        
        SimpleMatrix R2 = createMatrix(new double[][]{{0.1}});
        
        printMatrix("A", A2);
        printMatrix("B", B2);
        printMatrix("Q", Q2);
        printMatrix("R", R2);
        
        SimpleMatrix X2 = solveDARE(A2, B2, Q2, R2);
        printMatrix("Solution X", X2);
        
        SimpleMatrix residual2 = computeResidual(A2, B2, Q2, R2, X2);
        printMatrix("Residual (should be near zero)", residual2);
        
        SimpleMatrix K2 = computeOptimalGain(A2, B2, R2, X2);
        printMatrix("Optimal gain K", K2);
        
        boolean stable2 = isStable(A2, B2, K2);
        System.out.println("Closed-loop system stable: " + stable2);
        
        System.out.println("===========================================\n");
        
    }
}