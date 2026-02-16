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
                        // collect second spike line

                        .splineToLinearHeading(new Pose2d(17.5, 24,Math.toRadians(90)),Math.toRadians(90))

                        .lineToY(52)
                        .lineToY(45)
                        .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(134))
                        .strafeToLinearHeading(new Vector2d(10, 40), Math.toRadians(134))
                        .strafeToLinearHeading(new Vector2d(10, 54), Math.toRadians(134))
                        .waitSeconds(0.15)
                        .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(134))

                        // collect first spike line
                        .splineToLinearHeading(new Pose2d(-5, 24,Math.toRadians(90)),Math.toRadians(90))

                        .lineToY(48)
                        .lineToY(45)

//clear

                        // Shooter runs


                        .strafeToLinearHeading(new Vector2d(-29, 11.5), Math.toRadians(127))



                        // Shooter runs


                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}