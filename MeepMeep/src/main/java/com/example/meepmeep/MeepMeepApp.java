package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepApp {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // FTC autonomous start pose
        Pose2d startPose = new Pose2d( -57, 36, Math.toRadians(90));
        // Old code: always spline to (0, 0, 225)
        Pose2d splineTarget = new Pose2d(0, 0, Math.toRadians(225));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(startPose)
                        // First strafe and shoot
                        .setTangent(Math.toRadians(135))
                        .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(132))

                        // Shooter runs
                        // collect first spike line
                        .splineToLinearHeading(new Pose2d(-1.7, 24,Math.toRadians(97)),Math.toRadians(97))

                        .lineToY(48)
                        .lineToY(45)
                        .strafeToLinearHeading(new Vector2d(-12, 20), Math.toRadians(150))

                        // Shooter runs

                        // collect second spike line

                        .splineToLinearHeading(new Pose2d(18.5, 24,Math.toRadians(95)),Math.toRadians(95))
                        .lineToY(52)
                        .lineToY(45)

                        .strafeToLinearHeading(new Vector2d(-12, 20), Math.toRadians(150))



                        // collect third spike line

                        .strafeToLinearHeading(new Vector2d(37.5, 24), Math.toRadians(95))
                        .lineToY(52)
                        .lineToY(45)

                        .strafeToLinearHeading(new Vector2d(-29, 11.5), Math.toRadians(130))



                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}