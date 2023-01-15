package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SamAutoMM2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-35, -65, Math.toRadians(90));
        Pose2d stackPose = new Pose2d(-58.5,-12.7,Math.toRadians(180));//orig:180
        Pose2d farmPose = new Pose2d(-28.9,-5.5,Math.toRadians(45)); //42 original
        Pose2d approachPose = new Pose2d(-35.1,-13,Math.toRadians(90));
        //-60.8,-35.6
        Pose2d middlePark = new Pose2d(-35.8,-34.6,Math.toRadians(270));
        Pose2d leftPark =  new Pose2d(-60.8,-35.6,Math.toRadians(270));
        Pose2d rightPark =  new Pose2d(-10.8,-35.6,Math.toRadians(270));
        Pose2d beginnerPose = new Pose2d(-35.1,-13,Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 50, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(approachPose)
                                .lineToLinearHeading(farmPose)
                                .waitSeconds(1)
                                .lineToLinearHeading(approachPose)
                                .lineToSplineHeading(stackPose)
                                .waitSeconds(1)

                                .lineToSplineHeading(approachPose)
                                .lineToLinearHeading(farmPose)
                                .waitSeconds(1)
                                .lineToLinearHeading(approachPose)
                                .lineToSplineHeading(stackPose)
                                .waitSeconds(1)
                                .lineToSplineHeading(approachPose)
                                .lineToLinearHeading(farmPose)
                                .waitSeconds(1)
                                .lineToLinearHeading(approachPose)
                                .lineToSplineHeading(stackPose)
                                .waitSeconds(1)
                                .lineToSplineHeading(approachPose)
                                .lineToLinearHeading(farmPose)
                                .waitSeconds(1)
                                .lineToLinearHeading(approachPose)
                                .lineToSplineHeading(stackPose)
                                .waitSeconds(1)
                                .lineToSplineHeading(approachPose)
                                .lineToLinearHeading(farmPose)
                                .waitSeconds(1)
                                .lineToLinearHeading(approachPose)
                                .lineToSplineHeading(stackPose)
                                .waitSeconds(1).lineToSplineHeading(approachPose)
                                .lineToLinearHeading(farmPose)
                                .waitSeconds(1)
                                .lineToLinearHeading(approachPose)
                                .lineToSplineHeading(stackPose)
                                .waitSeconds(1)

                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
