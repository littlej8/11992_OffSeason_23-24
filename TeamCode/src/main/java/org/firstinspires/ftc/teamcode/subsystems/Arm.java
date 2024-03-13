package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Arm {
    private DcMotorEx arm;
    private CRServo grip;

    public static double ARM_POWER = 1.0, TELEOP_ARM_SPEED = 2;
    public static double DROP = -0.1, GRIP = 0.1;
    public static int PULLED_IN = 0, GRABBING = 88;

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        grip = hardwareMap.get(CRServo.class, "Claw");

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public class MoveArmToDrop implements Action {
        private boolean moving = false;

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!moving) {
                moving = true;

                arm.setTargetPosition(GRABBING);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(ARM_POWER);
            }

            return arm.isBusy();
        }
    }

    public Action moveArmToDrop() {
        return new MoveArmToDrop();
    }

    public class MoveArmIn implements Action {
        private boolean moving = false;

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!moving) {
                moving = true;

                arm.setTargetPosition(PULLED_IN);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(ARM_POWER);
            }

            return arm.isBusy();
        }
    }

    public Action moveArmIn() {
        return new MoveArmIn();
    }

    public class OpenClaw implements Action {
        @Override
        public boolean run(TelemetryPacket packet) {
            return true;
        }
    }
}