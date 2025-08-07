package Math;

import java.util.Arrays;
import java.util.List;

public class BezierCurve {
    public Matrix characteristicMatrix;
    public List<Vector> controlPoints;
    private int dimension;
    private int degree;

    public BezierCurve(Vector... controlPoints) {
        this.controlPoints = Arrays.stream(controlPoints).toList();
        degree = controlPoints.length - 1;
        dimension = controlPoints[0].getDimension();
        characteristicMatrix = buildBasisMatrix().multiply(buildControlPointMatrix());
    }

    private Matrix buildBasisMatrix() {
        Matrix M = new Matrix(degree + 1, degree + 1);
        for (int i = 0; i <= degree; i++) {
            for (int j = 0; j <= i; j++) {
                double entry = binomial(degree, j) * binomial(i, j) * Math.pow(-1, i - j);
                M.set(i, j, entry);
            }
        }

        return M;
    }

    private Matrix buildControlPointMatrix() {
        Matrix P = new Matrix(degree + 1, dimension);
        for (int i = 0; i <= degree; i++) {
            Vector p = controlPoints.get(i);
            for (int j = 0; j < dimension; j++) {
                P.set(i, j, p.get(j));
            }
        }
        return P;
    }

    public Vector evaluate(double t) {
        return getTVector(t).multiply(characteristicMatrix);
    }

    private long binomial(int n, int k) {
        if (k > n - k) k = n - k;
        long result = 1;
        for (int i = 1; i <= k; i++) {
            result *= n--;
            result /= i;
        }
        return result;
    }

    private Vector getTVector(double t) {
        double[] entries = new double[degree + 1];

        for (int i = 0; i < degree; i++) {
            entries[i] = Math.pow(t, degree - i);
        }

        return new Vector(entries);
    }

    private Vector getNthTVectorDerivative(double t, int n) {
        double[] entries = new double[degree + 1];

        for (int i = 0; i <= degree; i++) {
            int power = degree - i;

            if (power < n) {
                entries[i] = 0.0;
            } else {
                double coeff = MathFunctions.fallingFactorial(power, n);
                entries[i] = coeff * Math.pow(t, power - n);
            }
        }

        return new Vector(entries);
    }

    public Vector getDerivative(double t) {
        return getNthTVectorDerivative(t, 1).multiply(characteristicMatrix);
    }

    public Vector getTangentVector(double t) {
        return getDerivative(t);
    }

    public Vector getPrincipalNormalVector(double t) {
        Vector rPrime = getDerivative(t);         // r'(t)
        Vector rDoublePrime = getSecondDerivative(t); // r''(t)

        double speed = rPrime.magnitude();        // |r'(t)|
        if (speed == 0) {
            throw new ArithmeticException("Zero derivative magnitude at t = " + t);
        }

        // Compute T'(t) using: T' = [r'' * |r'| - r' * (r' · r'') / |r'|] / |r'|^2
        double dot = rPrime.dot(rDoublePrime);
        Vector term1 = rDoublePrime.scale(speed);
        Vector term2 = rPrime.scale(dot / speed);
        Vector Tprime = term1.subtract(term2).divide(speed * speed);

        // Return normalized T' as the principal normal vector
        return Tprime.normalize();
    }

    public Vector getSecondDerivative(double t) {
        return getNthTVectorDerivative(t, 2).multiply(characteristicMatrix);
    }

    public Polynomial getDotProductMinusPointDotDerivative(Vector point) {
        int n = dimension;
        int deg = degree;

        // Convert each coordinate function and its derivative to Polynomial
        Polynomial[] phi = new Polynomial[n];
        Polynomial[] dphi = new Polynomial[n];

        for (int dim = 0; dim < n; dim++) {
            double[] coeffs = new double[deg + 1];
            double[] dcoeffs = new double[deg]; // Derivative is 1 degree lower

            for (int i = 0; i <= deg; i++) {
                coeffs[deg - i] = characteristicMatrix.get(i, dim);  // reverse: highest degree first
            }

            for (int i = 1; i <= deg; i++) {
                dcoeffs[deg - i] = coeffs[deg - i + 1] * (deg - i + 1);
            }

            phi[dim] = new Polynomial(coeffs);
            dphi[dim] = new Polynomial(dcoeffs);
        }

        // Build f(t) = sum_i (phi_i(t) - p_i) * dphi_i(t)
        Polynomial result = new Polynomial(new double[]{0});
        for (int i = 0; i < n; i++) {
            Polynomial shifted = phi[i].add(-point.get(i)); // phi_i(t) - p_i
            Polynomial term = shifted.multiply(dphi[i]);          // * dphi_i(t)
            result = result.add(term);                            // accumulate
        }

        return result;
    }
}
