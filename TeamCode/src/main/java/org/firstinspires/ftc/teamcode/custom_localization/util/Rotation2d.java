package org.firstinspires.ftc.teamcode.custom_localization.util;

public class Rotation2d {
    double val, sin, cos;

    public Rotation2d(double rad) {
        val = rad;
        sin = Math.sin(rad);
        cos = Math.cos(rad);
    }

    public Rotation2d(double x, double y) {
        double magnitude = Math.hypot(x, y);
        sin = y / magnitude;
        cos = x / magnitude;
        val = Math.atan2(sin, cos);
    }

    public Rotation2d plus(Rotation2d other) {
        return rotateBy(other);
    }

    public Rotation2d minus(Rotation2d other) {
        return rotateBy(other.unaryMinus());
    }

    public Rotation2d unaryMinus() {
        return new Rotation2d(-val);
    }

    public Rotation2d rotateBy(Rotation2d other) {
        return new Rotation2d(
                cos * other.cos - sin * other.sin, cos * other.sin + sin * other.cos
        );
    }

    public double getRadians() {
        return val;
    }

    public double getDegrees() {
        return Math.toDegrees(val);
    }

    public double getCos() {
        return cos;
    }

    public double getSin() {
        return sin;
    }
}
