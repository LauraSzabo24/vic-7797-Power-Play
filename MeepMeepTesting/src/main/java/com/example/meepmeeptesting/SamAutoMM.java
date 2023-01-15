package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SamAutoMM {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(90));
        Pose2d farmingPose = new Pose2d(-30.4,-6,Math.toRadians(45));
        Pose2d approachPose = new Pose2d(-35.4,-11.3,Math.toRadians(45));
        Pose2d stackPose = new Pose2d(60, -12, Math.toRadians(0));

        //-60.8,-35.6
        Pose2d middlePark = new Pose2d(-35.8,-12,Math.toRadians(0));
        Pose2d leftPark =  new Pose2d(-57.8,-12,Math.toRadians(0));
        Pose2d rightPark =  new Pose2d(-10.8,-12,Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(63, 50, Math.toRadians(70), Math.toRadians(70), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(approachPose)
                                .lineToLinearHeading(farmingPose)
                                .waitSeconds(0.5)
                                .lineToLinearHeading(approachPose)
                                .lineToSplineHeading(stackPose)
                                .waitSeconds(1)
                                .lineToSplineHeading(approachPose)
                                .lineToLinearHeading(farmingPose)
                                .waitSeconds(0.5)
                                .lineToLinearHeading(approachPose)
                                .lineToSplineHeading(stackPose)
                                .waitSeconds(1)
                                .lineToSplineHeading(approachPose)
                                .lineToLinearHeading(farmingPose)
                                .waitSeconds(0.5)

                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
