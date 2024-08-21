package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "BareBonesLocalizationOpMode")
public class BareBonesLocalizationOpMode extends LinearOpMode {
    String[] motorNames = new String[]{"Backleft", "Frontleft", "Frontright", "Backright"};
    List<DcMotorEx> motors = new ArrayList<>();

    IMU imu;

    public static double STRAFE_MULT = 1.0;
    public static double TICKS_PER_REV = 1120;
    public static double WHEEL_DIAMETER = 4;
    public static double IN_PER_TICK = (WHEEL_DIAMETER * Math.PI) / TICKS_PER_REV;

    @Override
    public void runOpMode() {
        // bulk reading to improve loop times
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // init motors
        for (String motor : motorNames)
            motors.add(hardwareMap.get(DcMotorEx.class, motor));

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        motors.get(1).setDirection(DcMotor.Direction.REVERSE);
        motors.get(2).setDirection(DcMotor.Direction.REVERSE);

        // init imu
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // 0: fl
        // 1: fr
        // 2: bl
        // 3: br
        double[] prevWheels = new double[]{0, 0, 0, 0}, wheels = new double[]{0, 0, 0, 0};
        double heading = 0, prevHeading = 0;
        double[] curPose = new double[]{0.0, 0.0, 0.0}; // (x, y, heading (angle))

        boolean motorsMoving = false;

        while (!isStopRequested()) {
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double x = gamepad1.left_stick_x; // strafe
            double y = -gamepad1.left_stick_y; // forward
            double a = -gamepad1.right_stick_x; // turn

            // only write to motors if there is significant input
            if (Math.abs(x) > 0.25 || Math.abs(y) > 0.25 || Math.abs(a) > 0.25) {
                motorsMoving = true;

                // rotate input vector to align with the field
                double x0 = x;
                double y0 = y;
                x = (Math.cos(-heading) * x0) - (Math.sin(-heading) * y0);
                y = (Math.sin(-heading) * x0) + (Math.cos(-heading) * y0);

                // calculate wheel powers
                double[] p = new double[4];
                p[0] = -x + y - a;
                p[1] = x + y - a;
                p[2] = -x + y + a;
                p[3] = x + y + a;

                // keep motors below max power while sustaining the vector direction
                double max = Math.max(1, Math.max(Math.abs(p[0]), Math.max(Math.abs(p[1]), Math.max(Math.abs(p[2]), Math.abs(p[3])))));
                if (max > 1) for (int i = 0; i < 4; i++) p[i] /= max;
                for (int i = 0; i < 4; i++) motors.get(i).setPower(p[i]);
            } else if (motorsMoving) {
                // set powers to 0 if less than significant input
                
                motorsMoving = false;
                for (int i = 0; i < 4; i++) motors.get(i).setPower(0);
            }

            wheels[0] = motors.get(1).getCurrentPosition();
            wheels[1] = motors.get(2).getCurrentPosition();
            wheels[2] = motors.get(0).getCurrentPosition();
            wheels[3] = motors.get(3).getCurrentPosition();

            // get wheel deltas (changes) in inches
            double dfl = (wheels[0] - prevWheels[0]) * IN_PER_TICK;
            double dfr = (wheels[1] - prevWheels[1]) * IN_PER_TICK;
            double dbl = (wheels[2] - prevWheels[2]) * IN_PER_TICK;
            double dbr = (wheels[3] - prevWheels[3]) * IN_PER_TICK;

            // forward kinematics to convert wheel deltas to robot deltas
            double twistRobotY = (dfl + dfr + dbl + dbr) / 4;
            double twistRobotX = ((dbl + dfr - dfl - dbr) / 4) * STRAFE_MULT; // friction causes less movement in strafe
            double twistRobotTheta = heading - prevHeading;

            // set previous values to compare to next loop
            prevWheels[0] = wheels[0];
            prevWheels[1] = wheels[1];
            prevWheels[2] = wheels[2];
            prevWheels[3] = wheels[3];
            prevHeading = heading;

            // combine into double array to pass to exp()
            double[] twist = new double[]{twistRobotX, twistRobotY, twistRobotTheta};

            // integrate the twist (change in pose) into the current estimate of pose
            exp(curPose, twist);
            curPose[2] = heading;

            telemetry.addData("Position", "(%.2f, %.2f)", curPose[0], curPose[1]);
            telemetry.addData("Heading", "%.1f", Math.toDegrees(curPose[2]));
            telemetry.addData("Velocity", "(%.2f, %.2f)", twist[0], twist[1]);
            telemetry.addData("Angular Velocity", "%.1f", Math.toDegrees(twistRobotTheta));
            telemetry.update();
        }
    }

    // little bit of calculus to integrate with constant acceleration rather than velocity
    public static void exp(double[] cur, double[] twist) {
        double dx = twist[0];
        double dy = twist[1];
        double dtheta = twist[2];

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

        double[] transform = new double[]{
                dx * s - dy * c,
                dx * c + dy * s,
        };

        double curCos = Math.cos(-cur[2]);
        double curSin = Math.sin(-cur[2]);

        cur[0] -= transform[0] * curCos - transform[1] * curSin;
        cur[1] += transform[0] * curSin + transform[1] * curCos;
    }
}
