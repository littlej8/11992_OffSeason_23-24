package org.firstinspires.ftc.teamcode.custom_localization.dead_reckoning;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.custom_localization.util.Pose;
import org.firstinspires.ftc.teamcode.custom_localization.util.Rotation2d;

public class Odometry {
    private final MecanumKinematics kinematics;
    private Pose poseInches, velocityInches;

    private IMU gyro;
    private Rotation2d gyroOffset;
    private Rotation2d previousAngle;
    private final double[] previousWheelPositions;

    private boolean useIMU = true;
    private double strafe_mult = 1.0;

    public Odometry(
            MecanumKinematics kinematics,
            IMU imu,
            double[] wheelPositions,
            Pose initialPoseInches) {
        this.kinematics = kinematics;
        this.poseInches = initialPoseInches;
        this.gyro = imu;
        double gyroAngle = gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        this.gyroOffset = new Rotation2d(initialPoseInches.heading - gyroAngle);
        this.previousAngle = new Rotation2d(gyroAngle);
        this.previousWheelPositions = wheelPositions.clone();
    }

    public void setUseIMU(boolean useIMU) {
        this.useIMU = useIMU;
    }

    public void setStrafeMult(double s) {
        strafe_mult = s;
    }

    public void setPose(Pose pose) {
        poseInches = pose;
    }

    public Pose getPose() {
        return poseInches;
    }

    public Pose getVelocity() {
        return velocityInches;
    }

    public Pose update(double[] wheelPositions) {
        Rotation2d gyroAngle = new Rotation2d(gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        Rotation2d angle = (useIMU) ? gyroAngle.plus(gyroOffset) : new Rotation2d(poseInches.heading);

        Pose twist = kinematics.toTwist(previousWheelPositions, wheelPositions);
        twist.heading = angle.minus(previousAngle).getRadians();
        velocityInches = new Pose(twist.position.x * strafe_mult, twist.position.y, twist.heading);

        Pose newPose = poseInches.exp(twist);

        for (int i = 0; i < 4; i++)
            previousWheelPositions[i] = wheelPositions[i];

        previousAngle = angle;
        poseInches = new Pose(newPose.position, angle.getRadians());

        return poseInches;
    }
}
