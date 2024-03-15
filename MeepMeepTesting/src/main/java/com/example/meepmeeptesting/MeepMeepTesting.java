package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(160, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Action initial = myBot.getDrive().actionBuilder(new Pose2d(14.2, -62, 0))
                .splineToConstantHeading(new Vector2d(50, -30), 0)
                .waitSeconds(1)
                .lineToXConstantHeading(32)
                .waitSeconds(1)
                //.lineToXConstantHeading(35)
                .splineToConstantHeading(new Vector2d(30, -12), 0)
                .lineToXConstantHeading(-60, null, new ProfileAccelConstraint(-120, 120))
                .waitSeconds(1)
                .lineToXConstantHeading(35, null, new ProfileAccelConstraint(-120, 120))
                .splineToConstantHeading(new Vector2d(50, -30), 0)
                .waitSeconds(1)
                .build();

        Action cycle = myBot.getDrive().actionBuilder(new Pose2d(50, -30, 0))
                .lineToXConstantHeading(48)
                .splineToConstantHeading(new Vector2d(30, -12), 0)
                .lineToXConstantHeading(-60, null, new ProfileAccelConstraint(-120, 120))
                .waitSeconds(1)
                .lineToXConstantHeading(35, null, new ProfileAccelConstraint(-120, 120))
                .splineToConstantHeading(new Vector2d(50, -30), 0)
                .waitSeconds(1)
                .build();

        SequentialAction program = new SequentialAction(initial, cycle, cycle);

        myBot.runAction(program);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}