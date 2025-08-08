package ExampleUsage;

import Geometry.*;
import PathGeneration.Follower;
import PathGeneration.PIDController;
import PathGeneration.Path;

public class Example {
    public static void main(String[] args) {
        Follower follower = buildFollower();
        Path path = new Path()
                .setCurve(
                        new Vector(0,0,0),
                        new Vector(0,0,1),
                        new Vector(0,1,1),
                        new Vector(1,1,1)
                )
                .setHeadingInterpolation(c -> MathFunctions.linearInterpolation(
                    new Matrix.RotationMatrix(3),
                    new Matrix.RotationMatrix(new double[][]{
                            {0.1, 0.2, 0.3},
                            {0.4, 0.5, 0.6},
                            {0.7, 0.8, 0.9}
                    })
                ).apply(c.t()));
        follower.follow(path);

        do {
            follower.update();

        } while (!(follower.getPreviousTranslationalError().magnitude() < 0.1) ||
                !(follower.getPreviousRotationalError().frobeniusNorm() < 0.1) ||
                !(follower.getPreviousVelocity().positionalVelocity().magnitude() < 0.1) ||
                !(follower.getPreviousVelocity().rotationalVelocity().frobeniusNorm() < 0.1));
    }

    public static Follower buildFollower() {
        return new Follower.FollowerBuilder()
                .localizer(new Follower.Localizer() {
                    @Override
                    public Pose getPose() {
                        //getPose
                        return null;
                    }

                    @Override
                    public Fields.Twist getTwist() {
                        //getTwist
                        return null;
                    }
                })
                .driveRobot(f -> {
                    //Drive Robot
                })
                .maximumAcceleration(-90)
                .translationalController(new PIDController<>(0.1, 0.00001, 0.001) {
                    @Override
                    public double convertToScalarError(Vector current, Vector target) {
                        return MathFunctions.error(current, target);
                    }
                })
                .driveController(new PIDController<>(0.1, 0.00001, 0.001) {
                    @Override
                    public double convertToScalarError(Double current, Double target) {
                        return Math.abs(target - current);
                    }
                })
                .rotationalController(new PIDController<>(1, 0.001, 0.1) {
                    @Override
                    public double convertToScalarError(Matrix.RotationMatrix current, Matrix.RotationMatrix target) {
                        return 0;
                    }
                })
                .centripetalFeedforwardGain(0.005)
                .build();
    }
}
