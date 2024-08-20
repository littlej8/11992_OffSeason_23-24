package org.firstinspires.ftc.teamcode.custom_localization;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "BareBonesLocalizationOpMode")
public class BareBonesLocalizationOpMode extends LinearOpMode {
    String[] motorNames = new String[]{"Backleft", "Frontleft", "Frontright", "Backright"};
    List<DcMotorEx> motors = new ArrayList<>();

    IMU imu;

    public static double STRAFE_MULT = 1.0;

    @Override
    public void runOpMode() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        
        for (String motor : motorNames)
            motors.add(hardwareMap.get(DcMotorEx.class, motor));

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        motors.get(1).setDirection(DcMotor.Direction.REVERSE);
        motors.get(2).setDirection(DcMotor.Direction.REVERSE);

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
        double[] curPose = new double[]{0.0, 0.0, 0.0};

        while (!isStopRequested()) {
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
          
            wheels[0] = motors.get(1).getCurrentPosition();
            wheels[1] = motors.get(2).getCurrentPosition();
            wheels[2] = motors.get(0).getCurrentPosition();
            wheels[3] = motors.get(3).getCurrentPosition();

            double dfl = wheels[0] - prevWheels[0];
            double dfr = wheels[1] - prevWheels[1];
            double dbl = wheels[2] - prevWheels[2];
            double dbr = wheels[3] - prevWheels[3];
          
            double twistRobotY = (dfl + dfr + dbl + dbr) / 4;
            double twistRobotX = (dbl + dfr - dfl - dbr) / 4;
            double twistRobotTheta = heading - prevHeading;

            prevWheels[0] = wheels[0];
            prevWheels[1] = wheels[1];
            prevWheels[2] = wheels[2];
            prevWheels[3] = wheels[3];
            prevHeading = heading;

            double[] twist = new double[]{twistRobotX * STRAFE_MULT, twistRobotY};

            exp(curPose, twist);
            curPose[2] = heading;

            curPose[0] += fieldTwist[0];
            curPose[1] += fieldTwist[1];
            curPose[2] = heading;
            
            telemetry.addData("Position", "(%.2f, %.2f)", curPose[0], curPose[1]);
            telemetry.addData("Heading", "%.1f", Math.toDegrees(curPose[2]));
            telemetry.addData("Velocity", "(%.2f, %.2f)", twist[0], twist[1]);
            telemetry.addData("Angular Velocity", "%.1f", Math.toDegrees(twistRobotTheta));
            telemetry.update();
        }
    }

    public static void exp(double cur[], double[] twist) {
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

        double magnitude = Math.hypot(cosTheta, sinTheta);
        double sin = sinTheta / magnitude;
        double cos = cosTheta / magnitude;
        double rot = Math.atan2(sin, cos);

        double[] transform = new double[]{
                dx * s - dy * c,
                dx * c + dy * s,
                rot
        };
        
        double curCos = Math.cos(cur[2]);
        double curSin = Math.sin(cur[2]);
        
        cur[0] += transform[0] * curCos - transform[1] * curSin;
        cur[1] += transform[0] * curSin + transform[1] * curCos;
        cur[2] += transform[2];
    }
}
