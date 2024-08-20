package org.firstinspires.ftc.teamcode.custom_localization.util;

public class Pose {
    public Vector2D position;
    public double heading;

    public Pose(Vector2D position, double heading) {
        this.position = position;
        this.heading = heading;
    }

    public Pose(double x, double y) {
        this.position = new Vector2D(x, y);
        this.heading = 0;
    }

    public Pose(double x, double y, double h) {
        this.position = new Vector2D(x, y);
        this.heading = h;
    }

    public Pose plus(Pose other) {
        return transformBy(other);
    }

    public Pose transformBy(Pose other) {
        return new Pose(
                position.added(other.position.rotated(heading)),
                other.heading + heading
        );
    }

    public Pose exp(Pose twist) {
        // implementation from https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/geometry/Pose2d.java

        double dx = twist.position.x;
        double dy = twist.position.y;
        double dtheta = twist.heading;

        double sinTheta = Math.sin(dtheta);
        double cosTheta = Math.cos(dtheta);

        double s;
        double c;
        if (Math.abs(dtheta) < 1E-9) {
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            c = 0.5 * dtheta;
        } else {
            s = sinTheta / dtheta;
            c = (1 - cosTheta) / dtheta;
        }

        Pose transform = new Pose(
                dx * s - dy * c,
                dx * c + dy * s,
                (new Rotation2d(cosTheta, sinTheta)).getRadians()
        );
        return this.plus(transform);
    }
}
