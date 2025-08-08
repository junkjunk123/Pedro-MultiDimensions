package Geometry;

import PathGeneration.Path;

import java.util.function.Function;

public class MathFunctions {
    public static double fallingFactorial(int a, int b) {
        double result = 1;
        for (int i = 0; i < b; i++) {
            result *= (a - i);
        }
        return result;
    }

    public static double error(Vector current, Vector target) {
        return current.distance(target);
    }

    public static Matrix.SkewSymmetricMatrix error(Matrix.RotationMatrix current, Matrix.RotationMatrix target) {
        return target.multiply(current.transpose()).log();
    }

    public static Function<Double, Vector> linearInterpolation(Vector start, Vector end) {
        return t -> start.add(end.subtract(start).scale(t));
    }

    public static Path.HeadingInterpolation linearInterpolation(Matrix.RotationMatrix start, Matrix.RotationMatrix end) {
        // Relative rotation from start to end
        Matrix.RotationMatrix R_rel = start.transpose().multiply(end);

        // Skew-symmetric log of relative rotation
        Matrix.SkewSymmetricMatrix S = R_rel.log();

        return c -> start.multiply(S.scale(c.t()).exp());
    }
}
