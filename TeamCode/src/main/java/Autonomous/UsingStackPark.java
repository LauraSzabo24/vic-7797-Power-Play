package Autonomous;

//PID
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

//Servo
import com.qualcomm.robotcore.hardware.Servo;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import pipelines.AprilTagDetectionPipeline;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class UsingStackPark extends LinearOpMode
{
    //PID junk
    DcMotorEx pulleyMotorR;
    DcMotorEx pulleyMotorL;

    ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;
    private double integralSum =0;

    public static double Kp =0.0125;
    public static double Ki =0.0; //.00005
    public static double Kd =0.0;


    public static double targetPosition = 5;


    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    //PID ends here

    //CLAW goes here
    private Servo rightServo;
    private Servo leftServo;
    //CLAW ends here


    //added
    public static int tagNumber;

    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // Units are in pixels
    private static final double fx = 578.272;
    private static final double fy = 578.272;
    private static final double cx = 402.145;
    private static final double cy = 221.506;

    // UNITS ARE METERS
    private static final double tagsize = 0.166;

    //no idea what this is
    private int numFramesWithoutDetection = 0;

    private static final float DECIMATION_HIGH = 3;
    private static final float DECIMATION_LOW = 2;
    private static final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    private static final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
//

    public void initialize()
    {
        //PID initialization
        dashboard.setTelemetryTransmissionInterval(25);
        pulleyMotorL = hardwareMap.get(DcMotorEx.class, "LeftSlideMotor");
        pulleyMotorR = hardwareMap.get(DcMotorEx.class, "RightSlideMotor");
        pulleyMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        pulleyMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pulleyMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pulleyMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pulleyMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pulleyMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition = 0;

        //SERVO
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
    }

    //PID METHOD
    public double returnPower(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error -lastError)/ timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;

    }
    //PID ENDS HERE


    @Override
    public void runOpMode()
    {
        initialize();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);


        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                throw new RuntimeException("Error opening camera! Error code "+errorCode);
            }
        });

        waitForStart();

        telemetry.setMsTransmissionInterval(50);

        //from here2
        while (opModeIsActive())
        {
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            if(detections != null)
            {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                else
                {
                    numFramesWithoutDetection = 0;

                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : detections)
                    {
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));

                        //...
                        if(detection.id == 1 || detection.id == 2 || detection.id == 3)
                        {
                            tagNumber = detection.id;
                        }
                        //...
                    }
                }
                telemetry.update();
            }

            sleep(20);

            //PID CONSTANT CORRECTION OF SLIDES

            //PID ENDS HERE
        }

        double numberDetected = -1;

        while(!opModeIsActive()){
            // get the number of apriltag detected
            numberDetected = HauntedConePark.tagNumber;
        }

        //edited from here
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //roadrunner trajectory stuff
        Pose2d startPose = new Pose2d(0,0,0);
        ElapsedTime timer2 = new ElapsedTime();
        drive.setPoseEstimate(startPose);

        //trajectories and trajectory sequences
        TrajectorySequence goToThePole = drive.trajectorySequenceBuilder(startPose)
                .forward(69)
                .waitSeconds(2)
                .turn(90)//some amount, might be negative
                .waitSeconds(5)
                .build();
        TrajectorySequence dropCone = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .forward(5)
                .waitSeconds(2)
                .build();
        TrajectorySequence backwards = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(2)
                .back(5)
                .waitSeconds(1)
                .build();
        TrajectorySequence poleToStack = drive.trajectorySequenceBuilder(startPose)
                .turn(180)
                .waitSeconds(2)
                .forward(20)
                .waitSeconds(2)
                .build();

        TrajectorySequence goToParking = drive.trajectorySequenceBuilder(startPose)
                .back(6)
                .turn(90)
                .back(60)
                .build();
        Trajectory leftPark = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(12)
                .build();
        Trajectory rightPark = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(12)
                .build();

        waitForStart(); //also new

        while(opModeIsActive() && !isStopRequested()){
            //go to the tallest pole
            drive.followTrajectorySequence(goToThePole);

            //PID slide moving to drop cone
            targetPosition = 4200;
            moveSlides();

            //go forward and open claw
            drive.followTrajectorySequence(dropCone);
            rightServo.setPosition(0.5);
            leftServo.setPosition(0.5);

            //bring the slides lower
            targetPosition = 3000;
            moveSlides();

            double time = timer2.time();
            double currentTime = timer2.time();
            while(currentTime<(time+2))
            {
                currentTime = timer2.time();
            }
            //no clue if this timer works

            //bring the slides back up
            targetPosition = 4200;
            moveSlides();

            //close the claw
            //if timer above works will use here too
            rightServo.setPosition(0.25);
            leftServo.setPosition(0.75);

            //go back and lower slides
            drive.followTrajectorySequence(backwards);
            targetPosition = 0;
            moveSlides();

            //go to the stack
            drive.followTrajectorySequence(poleToStack);

            //open claw and lift up slides for top cone
            //also might use timer here
            rightServo.setPosition(0.5);
            leftServo.setPosition(0.5);
            targetPosition = 1000; //random number no clue if it's any good
            moveSlides();

            //go closer and close claw
            drive.followTrajectorySequence(dropCone);
            rightServo.setPosition(0.75);
            leftServo.setPosition(0.75);

            //backwards
            drive.followTrajectorySequence(backwards);

            //down with the slides
            //also might be able to skip this part depends on how stable
            time = timer2.time();
            currentTime = timer2.time();
            while(currentTime<(time+2))
            {
                currentTime = timer2.time();
            }//might not need timer
            //no clue if this timer works
            targetPosition = 0;
            moveSlides();

            drive.followTrajectorySequence(poleToStack);

            //repeat first part from here to place next cone---------------------

            PlaceMultipleCones(poleToStack, dropCone, backwards, drive, timer2); //repeat this as many times as needed

            //get in position to park
            drive.followTrajectorySequence(goToParking);

            if(numberDetected == 1){
                // park in zone 1
                telemetry.addData("PARK IN ZONE 1", numberDetected);
                drive.followTrajectory(leftPark);
            }
            else if(numberDetected == 2) {
                // park in zone 2
                telemetry.addData("PARK IN ZONE 2", numberDetected);
            }
            else {
                // park in zone 3
                telemetry.addData("PARK IN ZONE 3", numberDetected);
                drive.followTrajectory(rightPark);
            }
        }
    }

    public void PlaceMultipleCones(TrajectorySequence poleToStack, TrajectorySequence dropCone, TrajectorySequence backwards, SampleMecanumDrive drive, ElapsedTime timer2)
    {
        drive.followTrajectorySequence(poleToStack);

        //open claw and lift up slides for top cone
        //also might use timer here
        rightServo.setPosition(0.15);
        leftServo.setPosition(0.85);

        //timer
        time = timer2.time();
        double currentTime = timer2.time();
        while(currentTime<(time+2))
        {
            currentTime = timer2.time();
        }
        //no clue if this timer works

        targetPosition = 1000; //random number no clue if it's any good
        moveSlides();

        //go closer and close claw
        drive.followTrajectorySequence(dropCone);
        rightServo.setPosition(0.5);
        leftServo.setPosition(0.5);

        //backwards
        drive.followTrajectorySequence(backwards);

        //down with the slides
        //again, might need timer here
        //also might be able to skip this part depends on how stable
        targetPosition = 0;
        moveSlides();

        drive.followTrajectorySequence(poleToStack);
    }

    public void moveSlides()
    {
        while(!((targetPosition-pulleyMotorL.getCurrentPosition())>12))
        {
            double power = returnPower(targetPosition, pulleyMotorL.getCurrentPosition());
            pulleyMotorL.setPower(power);
            pulleyMotorR.setPower(power);
            targetPosition=pulleyMotorL.getCurrentPosition();

        }
    }
}
