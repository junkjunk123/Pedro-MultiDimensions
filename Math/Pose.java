package Math;

import Math.Matrix.RotationMatrix;

public class Pose {
    private Vector position;
    private RotationMatrix heading;

    public Pose(Vector position, RotationMatrix heading) {
        this.position = position;
        this.heading = heading;
    }

    public RotationMatrix getHeading() {
        return heading;
    }

    public Vector getPosition() {
        return position;
    }

    public void setHeading(RotationMatrix heading) {
        this.heading = heading;
    }

    public void setPosition(Vector position) {
        this.position = position;
    }
}
