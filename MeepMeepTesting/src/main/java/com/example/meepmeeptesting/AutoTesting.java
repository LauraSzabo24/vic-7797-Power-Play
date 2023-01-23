package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoTesting {
    public static double Kp = 0.0125;
    public static double Ki = 0.0; //.00005
    public static double Kd = 0.0;
    public static double Kf = 0.0;
    public static double smallHeight = 2100;
    public static double midHeight =3141;
    public static double tallHeight =3950;
    public static double grabHeight =720; //800
    public static double targetPosition = 0;
//sam is a submissive cat femboy-true

    public static double aPx = -35.1;//-37.1
    public static double aPy = -9.7;//-9.7

    public static double fPx = -30.2;//-30.2
    public static double fPy = -1.5;//-3.5 || -4.4

    public static double sPx = -58;//-55
    public static double sPy = -13.7; //-8.1 || -9.7

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-36, -62, Math.toRadians(90));
        Pose2d stackPose = new Pose2d(-58.5,-12.7,Math.toRadians(180));//orig:180
        Pose2d farmPose = new Pose2d(-31.9,-6.5,Math.toRadians(42)); //-33.9,-8.5,42 original
        Pose2d approachPose = new Pose2d(-37.1,-13,Math.toRadians(42));
        //
        Pose2d middlePark = new Pose2d(-35.8,-34.6,Math.toRadians(270));
        Pose2d leftPark =  new Pose2d(-60.8,-35.6,Math.toRadians(270));
        Pose2d rightPark =  new Pose2d(-10.8,-35.6,Math.toRadians(270));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)


                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(80), Math.toRadians(75), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(aPx-4, aPy+8, Math.toRadians(90)))
                                //.lineToLinearHeading(approachPose)
                                .lineToLinearHeading(new Pose2d(aPx-1, aPy-8, Math.toRadians(57)))
                                .lineToLinearHeading(new Pose2d(-30.5,-4.5,Math.toRadians(47))) //y:-2.5, heading: 45

                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {

                                })
                                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {


                                })
                                //.lineToLinearHeading(approachPose)
                                .lineToLinearHeading(new Pose2d(aPx-1, aPy-8, Math.toRadians(57)))

                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    targetPosition = grabHeight;
                                    // drive.setPoseEstimate(new Pose2d(aPx-1, aPy-2, Math.toRadians(57)));
                                })

                                //GRABSTACK1

                                .lineToLinearHeading(new Pose2d(-62.8,-9.7,Math.toRadians(180)))//-6.7
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                                    //closeClaw();


                                })
                                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> { // offset:-0.5
                                    targetPosition = tallHeight+100;
                                })
                                .lineToLinearHeading(approachPose)


                                //GOTOPOLE1
                                .lineToLinearHeading(new Pose2d(-29.1,-2.5,Math.toRadians(52))) //y:-1.8, heading:51 make this exactly on the pole new Pose2d(fPx,fPy,Math.toRadians(50))
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {

                                    targetPosition = tallHeight-200;

                                })
                                .lineToLinearHeading(approachPose)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> { //original offset = -0.5
                                    grabHeight -= 200;
                                    targetPosition = grabHeight;
                                    // drive.setPoseEstimate(new Pose2d(aPx-1, aPy-2, Math.toRadians(57)));
                                })
                                //GRABSTACK2

                                .lineToLinearHeading(new Pose2d(-63.8,-5.7,Math.toRadians(180)))//-11.7
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                                    //closeClaw();


                                })
                                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                                    targetPosition = tallHeight+100;
                                })
                                .lineToLinearHeading(approachPose)
                                //.lineToLinearHeading(new Pose2d(aPx-1, -5.7, Math.toRadians(57)))



                                //GOTOPOLE2
                                .lineToLinearHeading(new Pose2d(-29.1,-1,Math.toRadians(56))) //heading: 53  make this exactly on the pole new Pose2d(fPx,fPy,Math.toRadians(50))
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {


                                })
                                .lineToLinearHeading(approachPose)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> { //original offset = -0.5
                                    grabHeight -= 200;
                                    targetPosition = grabHeight;

                                })


                                //GRABSTACK3

                                .lineToLinearHeading(new Pose2d(-62.8,-3.7,Math.toRadians(180)))//y:-12
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {


                                })
                                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                                    targetPosition = tallHeight+100;
                                })
                                .lineToLinearHeading(approachPose)
                                //.lineToLinearHeading(new Pose2d(aPx-1, -5.7, Math.toRadians(57)))



                                //GOTOPOLE3
                                .lineToLinearHeading(new Pose2d(-29.1,-1,Math.toRadians(60))) //y: -0.5 heading: 59 make this exactly on the pole new Pose2d(fPx,fPy,Math.toRadians(50))
                                .waitSeconds(0.7)
                                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {



                                })
                                .lineToLinearHeading(approachPose)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> { //original offset = -0.5
                                    grabHeight -= 200;
                                    targetPosition = grabHeight;

                                })

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}