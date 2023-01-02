package Autonomous;

//PID

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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



@Autonomous
public class rotatingStaacks extends LinearOpMode {

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
    public static double midHeight =3141;
    public static double tallHeight =4200;
    public static double grabHeight =800; //940
    public static boolean liftIsBusy = false;
    public static double targetPosition = 0;


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
        PIDFController pidSlide = new PIDFController(Kp,Ki,Kd,Kf);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
         class Lifts{
             public void fixSlides()
             {
                 if(Math.abs(targetPosition - pulleyMotorL.getCurrentPosition()) > 12 && opModeIsActive()) //&& (4000>pulleyMotorL.getCurrentPosition()) && (-10<pulleyMotorL.getCurrentPosition()))
                 {

                    double power =  pidSlide.calculate(pulleyMotorL.getCurrentPosition(), targetPosition);
                    pulleyMotorL.setPower(power);
                    pulleyMotorR.setPower(power);
                    telemetry.addData("error:", Math.abs(targetPosition - pulleyMotorL.getCurrentPosition()));
                    telemetry.update();
                 }
             }
             public void closeClaw() {
                 rightServo.setPosition(0.5);
                 leftServo.setPosition(0.5);

             }
             public void openClaw() {
                 rightServo.setPosition(0.2);
                 leftServo.setPosition(0.8);
             }


         }
         Lifts slide = new Lifts();

        //
        initialize();
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

        waitForStart();

        telemetry.setMsTransmissionInterval(50);

        //from here2
        while (opModeInInit() && !(tagNumber == 1) && !(tagNumber == 2) && !(tagNumber == 3) && !(tagNumber == 4)) {
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

        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //Scoring Coordinates


        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(90));
        Pose2d stackPose = new Pose2d(-62.5, 7.5, Math.toRadians(183));
        Pose2d farmPose = new Pose2d(-30.5, 15.5, Math.toRadians(45));
        Pose2d approachPose = new Pose2d(-34.1, 6.7, Math.toRadians(90));
        //Parking Coordinates

        Pose2d middlePark = new Pose2d(-35.8,-34.6,Math.toRadians(270));
        Pose2d leftPark =  new Pose2d(-60.8,-35.6,Math.toRadians(270));
        Pose2d rightPark =  new Pose2d(-10.8,-35.6,Math.toRadians(270));

        drive.setPoseEstimate(startPose);


        //trajectories and trajectory sequences


        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(approachPose)
                .lineToLinearHeading(farmPose) //make exactly on pole
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                   targetPosition = tallHeight;
                })
                .forward(6)
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    slide.openClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {

                  targetPosition = grabHeight;
                })
                .lineToLinearHeading(farmPose)
                //.lineToLinearHeading(approachPose)




                //TRAJECTORY-PT-2



                .lineToLinearHeading(stackPose)
                .forward(6)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    //closeClaw();
                    slide.closeClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    targetPosition = 1400;
                })

                //.lineToLinearHeading(approachPose)
                .lineToLinearHeading(farmPose) //make this exactly on the pole
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    targetPosition = tallHeight;
                })
                .forward(6)
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    slide.openClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {

                   // targetPosition = grabHeight-250;
                    targetPosition = 0;
                })
                //grabHeight - 188
                .lineToLinearHeading(approachPose)
               // .lineToLinearHeading(stackPose)
               // .forward(7)

                .build();

        TrajectorySequence middleParking = drive.trajectorySequenceBuilder(Trajectory1.end())
                .lineToLinearHeading(middlePark)
                .build();



        waitForStart();


        slide.closeClaw();
        drive.followTrajectorySequenceAsync(Trajectory1);
        while(opModeIsActive())
        {
            drive.update();
            slide.fixSlides();
        }



    }


}