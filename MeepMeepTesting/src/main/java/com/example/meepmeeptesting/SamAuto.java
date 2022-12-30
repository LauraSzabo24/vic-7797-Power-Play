package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SamAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-35, -65, Math.toRadians(90));
        Pose2d midPose = new Pose2d(-34.5, -20, Math.toRadians(90));
        Pose2d approachPose = new Pose2d(-30.4,-6,Math.toRadians(42));
        //-60.8,-35.6
        Pose2d middlePark = new Pose2d(-35.8,-34.6,Math.toRadians(270));
        Pose2d leftPark =  new Pose2d(-60.8,-35.6,Math.toRadians(270));
        Pose2d rightPark =  new Pose2d(-10.8,-35.6,Math.toRadians(270));
        Pose2d beginnerPose = new Pose2d(-35.1,-13,Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 8)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(midPose)
                                .splineToSplineHeading(approachPose, Math.toRadians(45))
                                .waitSeconds(0.5)

                                .back(3)
                                .splineToLinearHeading(new Pose2d(-36.4,-18,Math.toRadians(130)), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(-46.4,-12,Math.toRadians(160)))
                                .lineToLinearHeading(new Pose2d(-59,-12,Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-46.4,-12,Math.toRadians(160)))
                                .lineToSplineHeading(new Pose2d(-36.4,-18,Math.toRadians(130)))
                                .splineToLinearHeading(approachPose, Math.toRadians(45))

                                .waitSeconds(0.5)
                                .back(3)
                                .splineToLinearHeading(new Pose2d(-36.4,-18,Math.toRadians(130)), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(-46.4,-12,Math.toRadians(160)))
                                .lineToLinearHeading(new Pose2d(-59,-12,Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-46.4,-12,Math.toRadians(160)))
                                .lineToSplineHeading(new Pose2d(-36.4,-18,Math.toRadians(130)))
                                .splineToLinearHeading(approachPose, Math.toRadians(45))

                                .waitSeconds(0.5)
                                .back(3)
                                .splineToLinearHeading(new Pose2d(-36.4,-18,Math.toRadians(130)), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(-46.4,-12,Math.toRadians(160)))
                                .lineToLinearHeading(new Pose2d(-59,-12,Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-46.4,-12,Math.toRadians(160)))
                                .lineToSplineHeading(new Pose2d(-36.4,-18,Math.toRadians(130)))
                                .splineToLinearHeading(approachPose, Math.toRadians(45))

                                .waitSeconds(0.5)
                                .back(3)
                                .splineToLinearHeading(new Pose2d(-36.4,-18,Math.toRadians(130)), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(-46.4,-12,Math.toRadians(160)))
                                .lineToLinearHeading(new Pose2d(-59,-12,Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-46.4,-12,Math.toRadians(160)))
                                .lineToSplineHeading(new Pose2d(-36.4,-18,Math.toRadians(130)))
                                .splineToLinearHeading(approachPose, Math.toRadians(45))

                                .lineToLinearHeading(new Pose2d(-35.4,-11,Math.toRadians(42)))
                                .lineToLinearHeading(new Pose2d(-35,-15,Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-35,-35,Math.toRadians(90)))
                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
