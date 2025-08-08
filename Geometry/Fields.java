package Geometry;

public class Fields {
    public record SpecialEuclideanTangentBundle(Twist twist, Matrix.RotationMatrix rotation) {
        public Matrix getRotationalComponent() {
            return rotation.multiply(twist.matrix());
        }

        public Vector getTranslationalComponent() {
            return twist.vector();
        }
    }

    public record Twist(Vector vector, Matrix.SkewSymmetricMatrix matrix) { }
}
