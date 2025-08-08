package PathGeneration;

import Geometry.*;
import Geometry.Fields.Twist;

import PathGeneration.GuidingVectorField.EuclideanGuidingVectorField;

import java.util.function.Consumer;

public class Follower {
    private Path path;
    private Controller<Vector> translationalController;
    private Controller<Matrix.RotationMatrix> rotationalController;
    private Controller<Double> driveController;
    private double centripetalFeedforwardGain;
    private final Localizer localizer;
    private double maximumAcceleration = 0;
    private Pose previousPose;
    private Twist previousVelocity;
    private Vector previousTranslationalError;
    private Matrix.SkewSymmetricMatrix previousRotationalError;

    public interface Localizer {
        Pose getPose();
        Twist getTwist();

        default Vector getVelocity() {
            return getTwist().positionalVelocity();
        }
    }

    public Consumer<GuidingVectorField<Fields.SpecialEuclideanTangentBundle>> hardwareAction;

    public Follower(Localizer localizer, Consumer<GuidingVectorField<Fields.SpecialEuclideanTangentBundle>> hardwareAction) {
        this.localizer = localizer;
        this.hardwareAction = hardwareAction;
    }

    public void setControllers(Controller<Vector> translationalController, Controller<Matrix.RotationMatrix> rotationalController,
                               Controller<Double> driveController, double centripetalFeedforwardGain) {
        this.translationalController = translationalController;
        this.rotationalController = rotationalController;
        this.driveController = driveController;
        this.centripetalFeedforwardGain = centripetalFeedforwardGain;
    }

    public void setCentripetalFeedforwardGain(double centripetalFeedforwardGain) {
        this.centripetalFeedforwardGain = centripetalFeedforwardGain;
    }

    public void setMaximumAcceleration(double maximumAcceleration) {
        this.maximumAcceleration = maximumAcceleration;
    }

    public void follow(Path path) {
        this.path = path;
        resetControllers();
    }

    public void update() {
        if (path == null) return;

        Pose currentPos = localizer.getPose();
        Vector currentVel = localizer.getVelocity();
        double psi = path.computeClosestTValue(currentPos.position());
        Pose targetPos = path.getPose(psi);

        GuidingVectorField<Fields.SpecialEuclideanTangentBundle> result = GuidingVectorField.compose(
                translationalVectorField(targetPos.position())
                        .add(driveVectorField(psi, currentVel))
                        .add(centripetalVectorField(psi, currentVel)),
                rotationalGuidingVectorField(targetPos),
                currentPos.orientation()
        );

        previousPose = currentPos;
        previousVelocity = localizer.getTwist();

        hardwareAction.accept(result);
    }

    public EuclideanGuidingVectorField translationalVectorField(Vector target) {
        return new GuidingVectorField.EuclideanGuidingVectorField() {
            @Override
            public Vector evaluate(Pose pose) {
                Vector current = pose.position();
                Vector translationalError = target.subtract(current);
                previousTranslationalError = translationalError.copy();
                translationalError.normalize();
                double controlOutput = translationalController.driveToState(current, target);
                return translationalError.scale(controlOutput);
            }
        };
    }

    public EuclideanGuidingVectorField driveVectorField(double t, Vector currentVel) {
        return new EuclideanGuidingVectorField() {
            @Override
            public Vector evaluate(Pose pose) {
                Vector tangent = path.getCurve().getTangentVector(t);
                Vector velocity = currentVel.projectOnto(tangent);
                double speedSquared = velocity.magnitudeSquared();
                double stoppingDistance = speedSquared / 2 * Math.abs(maximumAcceleration);

                if (path.getCurve().distanceRemaining(t) > stoppingDistance) {
                    return tangent;
                }

                double desiredSpeed = Math.sqrt(2 * Math.abs(maximumAcceleration) * stoppingDistance);
                double controlOutput = driveController.driveToState(Math.sqrt(speedSquared), desiredSpeed);

                return tangent.scale(controlOutput);
            }
        };
    }

    public EuclideanGuidingVectorField centripetalVectorField(double t, Vector velocity) {
        return new EuclideanGuidingVectorField() {
            @Override
            public Vector evaluate(Pose pose) {
                double speedSquared = velocity.magnitudeSquared();
                double scalar = centripetalFeedforwardGain * speedSquared / path.getCurve().getDerivative(t).magnitude();
                Vector normalVector = path.getCurve().getPrincipalNormalVector(t, false);
                return normalVector.scale(scalar);
            }
        };
    }

    public GuidingVectorField<Matrix.SkewSymmetricMatrix> rotationalGuidingVectorField(Pose targetPos) {
        return new GuidingVectorField<>() {
            @Override
            public Matrix.SkewSymmetricMatrix evaluate(Pose pose) {
                Matrix.RotationMatrix targetOrientation = targetPos.orientation();
                Matrix.RotationMatrix currentOrientation = pose.orientation();
                Matrix.SkewSymmetricMatrix headingError = MathFunctions.error(currentOrientation, targetOrientation);
                previousRotationalError = headingError.copy();
                double controlOutput = rotationalController.driveToState(currentOrientation, targetOrientation);
                return headingError.scale(controlOutput);
            }
        };
    }

    public static class FollowerBuilder {
        private Localizer localizer;
        private Consumer<GuidingVectorField<Fields.SpecialEuclideanTangentBundle>> hardwareAction;
        private Controller<Vector> translationalController;
        private Controller<Matrix.RotationMatrix> rotationalController;
        private Controller<Double> driveController;
        private double centripetalFeedforwardGain;
        private double maximumAcceleration;

        public FollowerBuilder localizer(Localizer localizer) {
            this.localizer = localizer;
            return this;
        }

        public FollowerBuilder driveRobot(Consumer<GuidingVectorField<Fields.SpecialEuclideanTangentBundle>> hardwareAction) {
            this.hardwareAction = hardwareAction;
            return this;
        }

        public FollowerBuilder translationalController(Controller<Vector> translationalController) {
            this.translationalController = translationalController;
            return this;
        }

        public FollowerBuilder rotationalController(Controller<Matrix.RotationMatrix> rotationalController) {
            this.rotationalController = rotationalController;
            return this;
        }

        public FollowerBuilder driveController(Controller<Double> driveController) {
            this.driveController = driveController;
            return this;
        }

        public FollowerBuilder centripetalFeedforwardGain(double centripetalFeedforwardGain) {
            this.centripetalFeedforwardGain = centripetalFeedforwardGain;
            return this;
        }

        public FollowerBuilder maximumAcceleration(double maximumAcceleration) {
            this.maximumAcceleration = maximumAcceleration;
            return this;
        }

        public Follower build() {
            Follower follower = new Follower(localizer, hardwareAction);
            follower.setControllers(translationalController, rotationalController, driveController, centripetalFeedforwardGain);
            follower.setCentripetalFeedforwardGain(centripetalFeedforwardGain);
            follower.setMaximumAcceleration(maximumAcceleration);
            return follower;
        }
    }

    public Twist getPreviousVelocity() {
        return previousVelocity;
    }

    public Pose getPreviousPose() {
        return previousPose;
    }

    public Matrix.SkewSymmetricMatrix getPreviousRotationalError() {
        return previousRotationalError;
    }

    public Vector getPreviousTranslationalError() {
        return previousTranslationalError;
    }

    public void breakFollowing() {
        this.path = null;
        resetControllers();
    }

    public void resetControllers() {
        this.driveController.reset();
        this.translationalController.reset();
        this.rotationalController.reset();
    }
}
