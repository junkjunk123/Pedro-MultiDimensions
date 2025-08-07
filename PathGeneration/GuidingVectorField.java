package PathGeneration;

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
}
