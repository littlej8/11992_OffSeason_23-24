package org.firstinspires.ftc.teamcode.custom_localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.custom_localization.dead_reckoning.MecanumKinematics;
import org.firstinspires.ftc.teamcode.custom_localization.dead_reckoning.Odometry;
import org.firstinspires.ftc.teamcode.custom_localization.util.Pose;
import org.firstinspires.ftc.teamcode.custom_localization.util.Vector2D;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "LocalizationOpMode")
public class LocalizationOpMode extends LinearOpMode {
    String[] motorNames = new String[]{"Backleft", "Frontleft", "Frontright", "Backright"};
    List<DcMotorEx> motors = new ArrayList<>();

    IMU imu;

    Odometry odometry;

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

        double[] wheelPositions = new double[4];
        wheelPositions[0] = motors.get(1).getCurrentPosition();
        wheelPositions[1] = motors.get(2).getCurrentPosition();
        wheelPositions[2] = motors.get(0).getCurrentPosition();
        wheelPositions[3] = motors.get(3).getCurrentPosition();

        MecanumKinematics kinematics = new MecanumKinematics(
                2,
                145.1,
                new Vector2D(7, 8), //fl
                new Vector2D(7, -8), //fr
                new Vector2D(-7, 8), //bl
                new Vector2D(-7, -8)  //br
        );
        odometry = new Odometry(
                kinematics,
                imu,
                wheelPositions,
                new Pose(0, 0, 0)
        );

        while (!isStopRequested()) {
            odometry.setStrafeMult(STRAFE_MULT);
            
            wheelPositions[0] = motors.get(1).getCurrentPosition();
            wheelPositions[1] = motors.get(2).getCurrentPosition();
            wheelPositions[2] = motors.get(0).getCurrentPosition();
            wheelPositions[3] = motors.get(3).getCurrentPosition();
            odometry.update(wheelPositions);

            Pose curPose = odometry.getPose();
            Pose curVel = odometry.getVelocity();

            telemetry.addData("Position", "(%.2f, %.2f)", curPose.position.x, curPose.position.y);
            telemetry.addData("Heading", "%.1f", Math.toDegrees(curPose.heading));
            telemetry.addData("Velocity", "(%.2f, %.2f)", curVel.position.x, curVel.position.y);
            telemetry.addData("Angular Velocity", "%.1f", Math.toDegrees(curVel.heading));
            telemetry.update();
        }
    }
}
