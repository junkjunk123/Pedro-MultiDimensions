package PathGeneration;

import java.util.List;
import java.util.function.Function;

import Geometry.*;

public class Path {
    public interface HeadingInterpolation extends Function<CurveContext, Matrix.RotationMatrix> { }
    public record CurveContext(double t, BezierCurve curve) {}
    private BezierCurve curve;
    private HeadingInterpolation headingInterpolation;

    public Path (BezierCurve curve, HeadingInterpolation headingInterpolation) {
        this.curve = curve;
        this.headingInterpolation = headingInterpolation;
    }

    public Path() {}

    public Pose getPose(double t) {
        return new Pose(curve.evaluate(t), headingInterpolation.apply(new CurveContext(t, curve)));
    }

    public double computeClosestTValue(Vector currentPose) {
        // Step 1: Construct f(t) = (ϕ(t) - p) · ϕ'(t)
        Polynomial distanceDerivative = curve.getDotProductMinusPointDotDerivative(currentPose);

        // Step 2: Find all real roots of f(t) in [0, 1]
        List<Double> candidates = Polynomial.PolynomialRootFinder.findRootsInInterval(distanceDerivative);

        // Step 3: Evaluate squared distance at each root + endpoints
        double minDistance = Double.POSITIVE_INFINITY;
        double bestT = 0.0;

        // Check interior roots
        for (double t : candidates) {
            Vector point = curve.evaluate(t);
            double distSq = point.subtract(currentPose).magnitudeSquared(); // Use squared norm for efficiency
            if (distSq < minDistance) {
                minDistance = distSq;
                bestT = t;
            }
        }

        // Check endpoints
        for (double t : new double[]{0.0, 1.0}) {
            Vector point = curve.evaluate(t);
            double distSq = point.subtract(currentPose).magnitudeSquared();
            if (distSq < minDistance) {
                minDistance = distSq;
                bestT = t;
            }
        }

        return bestT;
    }

    public BezierCurve getCurve() {
        return curve;
    }

    public HeadingInterpolation getHeadingInterpolation() {
        return headingInterpolation;
    }

    public Path setHeadingInterpolation(HeadingInterpolation headingInterpolation) {
        this.headingInterpolation = headingInterpolation;
        return this;
    }

    public Path setCurve(Vector... controlPoints) {
        this.curve = new BezierCurve(controlPoints);
        return this;
    }
}
