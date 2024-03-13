package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DroneLauncher {
    private Servo launcher;
    private boolean shot = false;

    public static double ShotPosition = 0.5, PrimePosition = 0.0;

    public DroneLauncher(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(Servo.class, "DroneLauncher");
    }

    public class ShootLauncher implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!shot) {
                launcher.setPosition(ShotPosition);
                shot = true;
            }

            packet.put("launcherPos", launcher.getPosition());

            return shot;
        }
    }

    public Action shootLauncher() {
        return new ShootLauncher();
    }

    public class ResetLauncher implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (shot) {
                launcher.setPosition(PrimePosition);
                shot = false;
            }

            packet.put("launcherPos", launcher.getPosition());

            return !shot;
        }
    }

    public Action resetLauncher() {
        return new ResetLauncher();
    }
}