package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class FunnyTeleMeepMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d terminalPose = new Pose2d(0, -52, Math.toRadians(270));
        Pose2d startPose = new Pose2d(0, -48, Math.toRadians(90));
        Pose2d farmingPose = new Pose2d(0,-32,Math.toRadians(90));
        //-60.8,-35.6

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(43, 50, Math.toRadians(60), Math.toRadians(60), 8)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)

                                .lineToLinearHeading(farmingPose)
                                .waitSeconds(0.5)
                                .back(15)
                                .turn(Math.toRadians(180))
                                .lineToLinearHeading(terminalPose)
                                .waitSeconds(0.5)
                                .back(5)
                                .turn(Math.toRadians(180))


                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
