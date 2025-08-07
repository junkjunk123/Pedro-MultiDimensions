package Geometry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * @param coeffs coeffs[i] = coefficient for t^i
 */
public record Polynomial(double[] coeffs) {
    public Polynomial(double[] coeffs) {
        this.coeffs = Arrays.copyOf(coeffs, coeffs.length);
    }

    public int degree() {
        return coeffs.length - 1;
    }

    public double evaluate(double t) {
        double result = 0;
        for (int i = degree(); i >= 0; i--) {
            result = result * t + coeffs[i];
        }
        return result;
    }

    public Polynomial derivative() {
        if (coeffs.length == 1) return new Polynomial(new double[]{0});
        double[] deriv = new double[coeffs.length - 1];
        for (int i = 1; i < coeffs.length; i++) {
            deriv[i - 1] = i * coeffs[i];
        }
        return new Polynomial(deriv);
    }

    // Reparametrize this polynomial onto a subinterval [tMin, tMax]
    public Polynomial reparametrize(double tMin, double tMax) {
        // Change of variable: t = tMin + (tMax - tMin) * u, u in [0, 1]
        // We apply the substitution and re-expand into power basis
        Polynomial result = new Polynomial(new double[]{0});
        for (int i = 0; i < coeffs.length; i++) {
            Polynomial term = powAffine(i, tMin, tMax - tMin).scale(coeffs[i]);
            result = result.add(term);
        }
        return result;
    }

    public Polynomial add(Polynomial other) {
        int maxDeg = Math.max(this.degree(), other.degree());
        double[] sum = new double[maxDeg + 1];
        for (int i = 0; i <= maxDeg; i++) {
            double a = i < this.coeffs.length ? this.coeffs[i] : 0;
            double b = i < other.coeffs.length ? other.coeffs[i] : 0;
            sum[i] = a + b;
        }
        return new Polynomial(sum);
    }

    public Polynomial scale(double scalar) {
        double[] scaled = new double[coeffs.length];
        for (int i = 0; i < coeffs.length; i++) {
            scaled[i] = coeffs[i] * scalar;
        }
        return new Polynomial(scaled);
    }

    // Returns the polynomial (tMin + tScale * t)^k
    public static Polynomial powAffine(int k, double tMin, double tScale) {
        Polynomial result = new Polynomial(new double[]{1.0});
        Polynomial affine = new Polynomial(new double[]{tMin, tScale});
        for (int i = 0; i < k; i++) {
            result = result.multiply(affine);
        }
        return result;
    }

    public Polynomial multiply(Polynomial other) {
        double[] product = new double[this.degree() + other.degree() + 1];
        for (int i = 0; i < this.coeffs.length; i++) {
            for (int j = 0; j < other.coeffs.length; j++) {
                product[i + j] += this.coeffs[i] * other.coeffs[j];
            }
        }
        return new Polynomial(product);
    }

    public static Polynomial fromRoots(double[] roots) {
        Polynomial result = new Polynomial(new double[]{1});
        for (double r : roots) {
            result = result.multiply(new Polynomial(new double[]{-r, 1}));
        }
        return result;
    }

    public Polynomial add(double c) {
        double[] result = Arrays.copyOf(coeffs, coeffs.length);
        result[0] += c;
        return new Polynomial(result);
    }

    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (int i = coeffs.length - 1; i >= 0; i--) {
            if (Math.abs(coeffs[i]) < 1e-10) continue;
            if (!sb.isEmpty()) sb.append(" + ");
            sb.append(String.format("%.4f", coeffs[i]));
            if (i > 0) sb.append("t");
            if (i > 1) sb.append("^").append(i);
        }
        return !sb.isEmpty() ? sb.toString() : "0";
    }

    public static class PolynomialRootFinder {
        private static final double EPS = 1e-6;
        private static final int MAX_DEPTH = 30;

        public static List<Double> findRootsInInterval(Polynomial poly) {
            List<Double> roots = new ArrayList<>();
            findRootsRecursive(poly, 0.0, 1.0, roots, 0);
            return roots;
        }

        private static void findRootsRecursive(Polynomial poly, double tMin, double tMax, List<Double> roots, int depth) {
            if (depth > MAX_DEPTH) return;

            int signChanges = countSignChanges(poly.coeffs);
            if (signChanges == 0) return;

            if (signChanges == 1) {
                double root = bisection(poly, tMin, tMax);
                if (!Double.isNaN(root)) roots.add(root);
                return;
            }

            double tMid = (tMin + tMax) / 2.0;
            Polynomial left = poly.reparametrize(tMin, tMid);
            Polynomial right = poly.reparametrize(tMid, tMax);

            findRootsRecursive(left, tMin, tMid, roots, depth + 1);
            findRootsRecursive(right, tMid, tMax, roots, depth + 1);
        }

        private static double bisection(Polynomial poly, double a, double b) {
            double fa = poly.evaluate(a);
            double fb = poly.evaluate(b);

            if (fa * fb > 0) return Double.NaN;

            for (int i = 0; i < 50; i++) {
                double m = 0.5 * (a + b);
                double fm = poly.evaluate(m);
                if (Math.abs(fm) < EPS) return m;
                if (fa * fm < 0) {
                    b = m;
                    fb = fm;
                } else {
                    a = m;
                    fa = fm;
                }
            }
            return 0.5 * (a + b);
        }

        private static int countSignChanges(double[] coeffs) {
            int changes = 0;
            double prev = 0;
            for (double c : coeffs) {
                if (Math.abs(c) < EPS) continue;
                if (prev != 0 && c * prev < 0) changes++;
                prev = c;
            }
            return changes;
        }
    }
}