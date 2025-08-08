package PathGeneration;

import Geometry.Fields;
import Geometry.Matrix;
import Geometry.Pose;
import Geometry.Vector;

import java.util.function.Function;

public abstract class GuidingVectorField<T> {
    public static abstract class EuclideanGuidingVectorField extends GuidingVectorField<Vector> {
        public EuclideanGuidingVectorField add(EuclideanGuidingVectorField other) {
            Function<Pose, Vector> evaluate = this::evaluate;

            return new EuclideanGuidingVectorField() {
                @Override
                public Vector evaluate(Pose pose) {
                    return evaluate.apply(pose).add(other.evaluate(pose));
                }
            };
        }
    }

    public abstract T evaluate(Pose pose);

    public static GuidingVectorField<Fields.SpecialEuclideanTangentBundle> compose(
            EuclideanGuidingVectorField a,
            GuidingVectorField<Matrix.SkewSymmetricMatrix> b,
            Matrix.RotationMatrix rot
    )
    {
        return new GuidingVectorField<>() {
            @Override
            public Fields.SpecialEuclideanTangentBundle evaluate(Pose pose) {
                return new Fields.SpecialEuclideanTangentBundle(new Fields.Twist(a.evaluate(pose),b.evaluate(pose)), rot);
            }
        };
    }
}
