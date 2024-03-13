package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Trapdoor {
    private Servo trapdoor;
    private boolean open = false;

    public static double OpenPosition = 1.0, ClosedPosition = 0.75;

    public Trapdoor(HardwareMap hardwareMap) {
        trapdoor = hardwareMap.get(Servo.class, "Trapdoor");
        trapdoor.setDirection(Servo.Direction.REVERSE);
    }

    public class OpenTrapdoor implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!open) {
                trapdoor.setPosition(OpenPosition);
                open = true;
            }

            packet.put("trapdoorPos", trapdoor.getPosition());
            return open;
        }
    }

    public Action openTrapdoor() {
        return new OpenTrapdoor();
    }

    public class CloseTrapdoor implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (open) {
                trapdoor.setPosition(ClosedPosition);
                open = false;
            }

            packet.put("trapdoorPos", trapdoor.getPosition());

            return !open;
        }
    }

    public Action closeTrapdoor() {
        return new CloseTrapdoor();
    }
}