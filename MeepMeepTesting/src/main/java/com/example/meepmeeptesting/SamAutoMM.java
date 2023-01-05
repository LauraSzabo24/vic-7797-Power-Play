package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SamAutoMM {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(90));
        Pose2d midPose = new Pose2d(-34.5, -20, Math.toRadians(90));
        Pose2d farmingPose = new Pose2d(-30.4,-6,Math.toRadians(45));
        //-60.8,-35.6
        Pose2d middlePark = new Pose2d(-35.8,-12,Math.toRadians(0));
        Pose2d leftPark =  new Pose2d(-57.8,-12,Math.toRadians(0));
        Pose2d rightPark =  new Pose2d(-10.8,-12,Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 50, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                                    //bring up slides
                                })
                                .lineToLinearHeading(midPose)
                                .splineToSplineHeading(farmingPose, Math.toRadians(45))
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{
                                    //lower slides interval
                                    //open claw
                                })

                                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                                    //bring down slides-interval
                                })
                                .back(3)
                                .splineToSplineHeading(new Pose2d(-59,-12,Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(.5)
                                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{
                                    //close claw lift
                                })
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-30.4,-6,Math.toRadians(45)), Math.toRadians(45))
                                .setReversed(false)
                                .waitSeconds(0.5)

                                .back(3)
                                .splineToSplineHeading(new Pose2d(-59,-12,Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(.5)
                                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{
                                    //close claw lift
                                })
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-30.4,-6,Math.toRadians(45)), Math.toRadians(45))
                                .setReversed(false)
                                .waitSeconds(0.5)

                                .back(3)
                                .splineToSplineHeading(new Pose2d(-59,-12,Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(.5)
                                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{
                                    //close claw lift
                                })
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-30.4,-6,Math.toRadians(45)), Math.toRadians(45))
                                .setReversed(false)
                                .waitSeconds(0.5)

                                .back(3)
                                .splineToSplineHeading(new Pose2d(-59,-12,Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(.5)
                                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{
                                    //close claw lift
                                })
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-30.4,-6,Math.toRadians(45)), Math.toRadians(45))
                                .setReversed(false)
                                .waitSeconds(0.5)

                                .back(3)
                                .splineToSplineHeading(new Pose2d(-59,-12,Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(.5)
                                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{
                                    //close claw lift
                                })
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-30.4,-6,Math.toRadians(45)), Math.toRadians(45))
                                .setReversed(false)
                                .waitSeconds(0.5)

                                .waitSeconds(0.5)
                                .setReversed(true)
                                .back(3)
                                .splineToSplineHeading((rightPark),Math.toRadians(0))
                                .setReversed(false)
                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
