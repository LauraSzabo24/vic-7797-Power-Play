package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class FunnyTeleMeepMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d terminalPose = new Pose2d(0, -55, Math.toRadians(270));
        Pose2d startPose = new Pose2d(-10.8,-35.6,Math.toRadians(270));
        Pose2d highPose = new Pose2d(0,-32,Math.toRadians(90));
        Pose2d midPose = new Pose2d(0,-35.6,Math.toRadians(270));
        Pose2d midRightPose = new Pose2d(17,-30,Math.toRadians(45));
        Pose2d midLeftPose = new Pose2d(-17,-30,Math.toRadians(135));
        //-60.8,-35.6

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(53, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)






                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
