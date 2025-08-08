package Geometry;

public class Fields {
    public record SpecialEuclideanTangentBundle(Twist twist, Matrix.RotationMatrix rotation) {
        public Matrix getRotationalComponent() {
            return rotation.multiply(twist.rotationalVelocity());
        }

        public Vector getTranslationalComponent() {
            return twist.positionalVelocity();
        }
    }

    public record Twist(Vector positionalVelocity, Matrix.SkewSymmetricMatrix rotationalVelocity) { }
}
