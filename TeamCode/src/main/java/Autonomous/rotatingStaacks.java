package Autonomous;

//PID

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


import pipelines.AprilTagDetectionPipeline;


@Config
@Disabled
@Autonomous
public class rotatingStaacks extends LinearOpMode {
    enum State {
        TO_POLE,
        TO_STACK,
        PARKING,//
        IDLE
    }

    //PID junk
    DcMotorEx pulleyMotorR;
    DcMotorEx pulleyMotorL;

    //timers
    ElapsedTime timer2 = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    //stop
    int stop = 0;

    private double lastError = 0;
    private double integralSum = 0;

    public static double Kp = 0.0125;
    public static double Ki = 0.0; //.00005
    public static double Kd = 0.0;
    public static double Kf = 0.0;
    public static double smallHeight = 2100;
    public static double midHeight = 3141;
    public static double tallHeight = 3950;
    public static double grabHeight = 720; //800
    public static boolean liftIsBusy = false;
    public static double targetPosition = 0;
//sam is a submissive cat femboy-true

    public static double aPx = -35.1;//-37.1
    public static double aPy = -9.7;//i am so funny & indra is dumb fr

    public static double fPx = -30.2;//-30.2
    public static double fPy = -2.8;//-3.5 || -4.4

    public static double sPx = -55;//-63.8
    public static double sPy = -9.7; //-8.1 || -9.7

    private static double offsetHead = 0;


    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    //PID ends here

    //CLAW goes here
    private Servo rightServo;
    private Servo leftServo;
    //CLAW ends here


    //added
    public static int tagNumber;
    private static int cycle = 0;

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

    public void initialize() {
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

        tagNumber = 0;

        closeClaw();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);


        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                throw new RuntimeException("Error opening camera! Error code " + errorCode);
            }
        });

    }

    //PID METHOD
    public double returnPower(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;

    }
    //PID ENDS HERE


    @Override
    public void runOpMode() {
        //


        //
        initialize();

        //waitForStart();

        telemetry.setMsTransmissionInterval(50);

        //from here2
        while (opModeInInit()) {
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            if (detections != null) {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                } else {
                    numFramesWithoutDetection = 0;

                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for (AprilTagDetection detection : detections) {
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));

                        //...
                        if (detection.id == 1 || detection.id == 2 || detection.id == 3) {
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


        //Trajectory starts here
        PIDFController pidSlide = new PIDFController(Kp, Ki, Kd, Kf);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //Scoring Coordinates


        Pose2d approachPose = new Pose2d(aPx - 1, aPy, Math.toRadians(57));//heading orgin:47
        Pose2d startPose = new Pose2d(-36, -62, Math.toRadians(90));
        Pose2d farmPose = new Pose2d(fPx, fPy, Math.toRadians(57));
        Pose2d stackPose = new Pose2d(sPx, sPy, Math.toRadians(180));


        Pose2d middlePark = new Pose2d(-35, -33.2, Math.toRadians(90));
        Pose2d leftPark = new Pose2d(-60.8, -33.2, Math.toRadians(90));
        Pose2d rightPark = new Pose2d(-11.8, -33.2, Math.toRadians(90));

        drive.setPoseEstimate(startPose);


        //trajectories and trajectory sequences


        drive.setPoseEstimate(startPose);
        TrajectorySequence bigTrajectory = drive.trajectorySequenceBuilder(startPose)
                //FIRST CONE
                .lineToLinearHeading(approachPose)
                .lineToLinearHeading(new Pose2d(-30.7, -4.5, Math.toRadians(45))) //make exactly on pole  public static double fPx = -30.2;//-30.2

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-4.5, () -> {
                    targetPosition = tallHeight;
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    for (int i = 0; i < 130; i++) openClaw();
                    targetPosition = tallHeight - 200;

                })
                .lineToLinearHeading(approachPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    targetPosition = grabHeight;
                })

                //GRABSTACK1

                .lineToLinearHeading(stackPose)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    //closeClaw();
                    closeClaw();

                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    targetPosition = tallHeight;
                })
                .lineToLinearHeading(approachPose)

                //GOTOPOLE1
                .lineToLinearHeading(farmPose) //make this exactly on the pole new Pose2d(fPx,fPy,Math.toRadians(50))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    for (int i = 0; i < 130; i++) openClaw();
                    targetPosition = tallHeight - 200;

                })
                .lineToLinearHeading(approachPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { //original offset = -0.5
                    targetPosition = grabHeight;
                })
                //GRABSTACK2

                .lineToLinearHeading(stackPose)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    //closeClaw();
                    closeClaw();

                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    targetPosition = tallHeight;
                })
                .lineToLinearHeading(approachPose)

                //GOTOPOLE2
                .lineToLinearHeading(farmPose) //make this exactly on the pole new Pose2d(fPx,fPy,Math.toRadians(50))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    for (int i = 0; i < 130; i++) openClaw();
                    targetPosition = tallHeight - 200;

                })
                .lineToLinearHeading(approachPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { //original offset = -0.5
                    targetPosition = grabHeight;
                })
                .build();


        TrajectorySequence zone1 = drive.trajectorySequenceBuilder(bigTrajectory.end())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    closeClaw();
                    targetPosition = 50;
                })
                .lineToLinearHeading(new Pose2d(-35.4, -11, Math.toRadians(42)))
                .lineToLinearHeading(middlePark)
                .lineToLinearHeading(leftPark)

                .build();

        TrajectorySequence zone2 = drive.trajectorySequenceBuilder(bigTrajectory.end())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    closeClaw();
                    targetPosition = 50;
                })
                .lineToLinearHeading(new Pose2d(-35.4, -11, Math.toRadians(42)))
                .lineToLinearHeading(middlePark)
                .build();

        TrajectorySequence zone3 = drive.trajectorySequenceBuilder(bigTrajectory.end())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {

                    closeClaw();
                    targetPosition = 50;

                })
                .lineToLinearHeading(new Pose2d(-35.4, -11, Math.toRadians(42)))
                .lineToLinearHeading(middlePark)
                .lineToLinearHeading(rightPark)

                .build();


        cycle = 0;
        grabHeight = 800;
        closeClaw();

        drive.followTrajectorySequenceAsync(bigTrajectory);
        State currentState = State.TO_POLE;

        while (opModeIsActive()) {
            switch (currentState) {
                case PARKING:
                    if (!drive.isBusy()) {
                        switch (tagNumber) {
                            case 1:
                                drive.followTrajectorySequenceAsync(zone1);
                                currentState = State.IDLE;
                                break;
                            case 2:
                                drive.followTrajectorySequenceAsync(zone2);
                                currentState = State.IDLE;
                                break;
                               case 3:
                                drive.followTrajectorySequenceAsync(zone3);
                                currentState = State.IDLE;
                                break;
                        }
                    }
                break;
                case IDLE:
                    break;
            }




        }
        drive.update();
        fixSlides();//pp
        telemetry.addData("offset", fPy);
        telemetry.update();
    }



    public void fixSlides()
    {
        if(Math.abs(targetPosition - pulleyMotorL.getCurrentPosition()) > 12 && opModeIsActive()) //&& (4000>pulleyMotorL.getCurrentPosition()) && (-10<pulleyMotorL.getCurrentPosition()))
        {

            double power = returnPower(targetPosition, pulleyMotorL.getCurrentPosition());
            //purely experimental
            pulleyMotorL.setPower(power);
            pulleyMotorR.setPower(power);
            telemetry.addData("positionLL:", pulleyMotorL.getCurrentPosition());
            telemetry.update();

        }
        else {
            pulleyMotorL.setPower(0);
            pulleyMotorR.setPower(0);
        }
    }
    public void closeClaw() {
        rightServo.setPosition(0.5);
        leftServo.setPosition(0.5);

    }
    public void openClaw() {
        rightServo.setPosition(0.20);//0.3
        leftServo.setPosition(0.80);//0.7
    }


}
