package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;

public class TankTeleOp extends LinearOpMode {
    List<DcMotorEx> left, right;

    @Override
    public void runOpMode() {
        left.add(hardwareMap.get(DcMotorEx.class, "fl"));
        left.add(hardwareMap.get(DcMotorEx.class, "bl"));
        right.add(hardwareMap.get(DcMotorEx.class, "fr"));
        right.add(hardwareMap.get(DcMotorEx.class, "br"));

        left.forEach(m -> m.setMode(DcMotor.RunMode.RUN_USING_ENCODER));
        right.forEach(m -> m.setMode(DcMotor.RunMode.RUN_USING_ENCODER));

        waitForStart();

        while (opModeIsActive()) {
            double l = -Math.pow(gamepad1.left_stick_y, 3);
            double r = -Math.pow(gamepad1.right_stick_y, 3);

            left.forEach(m -> m.setPower(l));
            right.forEach(m -> m.setPower(r));
        }
    }
}
