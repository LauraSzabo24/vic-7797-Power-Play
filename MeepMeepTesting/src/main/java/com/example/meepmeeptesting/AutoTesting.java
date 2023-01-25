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
                                .lineToLinearHeading(approachPose)
                                .lineToLinearHeading(approachPose.plus(new Pose2d(0,-20,-42)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}