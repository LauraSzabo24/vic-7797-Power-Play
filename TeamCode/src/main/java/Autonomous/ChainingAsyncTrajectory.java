package Autonomous;

//PID

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
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
public class ChainingAsyncTrajectory extends LinearOpMode {

    //FSM states
    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        PARKING_MIDDLE,   // Then, follow a lineTo() trajectory
        PARKING_LEFT,         // Then we want to do a point turn
        PARKING_RIGHT,   // Then, we follow another lineTo() trajectory
        IDLE            // Our bot will enter the IDLE state when done
    }
    //PID junk
    DcMotorEx pulleyMotorR;
    DcMotorEx pulleyMotorL;

    //timers

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
    public static double tallHeight =4115;
    public static double grabHeight =940;

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
        initialize();
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
                else
                {
                    pulleyMotorL.setPower(0);
                    pulleyMotorR.setPower(0);

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
        while (opModeIsActive() && !(tagNumber == 1) && !(tagNumber == 2) && !(tagNumber == 3) && !(tagNumber == 4)) {
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

        double numberDetected = -1;

        while (!opModeIsActive()) {
            // get the number of apriltag detected
            numberDetected = tagNumber;
        }

        //edited from here

        //roadrunner trajectory stuff


        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(90));
        Pose2d stackPose = new Pose2d(-62.5, 10.5, Math.toRadians(183));
        Pose2d farmPose = new Pose2d(-31.9, 15.5, Math.toRadians(48));
        Pose2d approachPose = new Pose2d(-34.1, 6.7, Math.toRadians(90));
        //Parking coordinates

        Pose2d middlePark = new Pose2d(-35.8,-25.6,Math.toRadians(90)); //-35.8 and -34.6
        Pose2d leftPark =  new Pose2d(-56.8,-25.6,Math.toRadians(90));
        Pose2d rightPark =  new Pose2d(-6.8,-25.6,Math.toRadians(90));

        drive.setPoseEstimate(startPose);


        //trajectories and trajectory sequences


        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    slide.closeClaw();
                })
                .lineToLinearHeading(approachPose)
                .lineToLinearHeading(farmPose) //make exactly on pole
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-4, () -> {
                    targetPosition = midHeight;
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    targetPosition = tallHeight;
                })
                .forward(6)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    slide.openClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    targetPosition = grabHeight-150;
                })
                .splineToLinearHeading(farmPose,Math.toRadians(48))





                //TRAJECTORY-PT-2



                .lineToLinearHeading(stackPose)
                .forward(6)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    slide.closeClaw();
                })
                .lineToLinearHeading(farmPose) //make this exactly on the pole
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-5, () -> {
                    targetPosition = smallHeight;
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    targetPosition = tallHeight + 100;
                })
                .forward(6.5)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    slide.openClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset( 0, () -> {
                     targetPosition = grabHeight-250;
                })
                .lineToLinearHeading(approachPose)

                .build();


        TrajectorySequence middleParking = drive.trajectorySequenceBuilder(Trajectory1.end())
                .lineToLinearHeading(middlePark)
                .build();

        TrajectorySequence leftParking = drive.trajectorySequenceBuilder(Trajectory1.end())
                .lineToLinearHeading(middlePark)
                .lineToLinearHeading(leftPark)
                .build();

        TrajectorySequence rightParking = drive.trajectorySequenceBuilder(Trajectory1.end())
                .lineToLinearHeading(middlePark)
                .lineToLinearHeading(rightPark)
                .build();






        waitForStart();
        slide.closeClaw();
        drive.followTrajectorySequenceAsync(Trajectory1);
        State currentState = State.TRAJECTORY_1;

        while(opModeIsActive())
        {
            switch (currentState) {
                case TRAJECTORY_1:
                    // Check if the drive class isn't busy
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        if(tagNumber == 2){

                            drive.followTrajectorySequenceAsync(middleParking);
                            currentState = State.IDLE;
                        }
                        if(tagNumber == 1){

                            drive.followTrajectorySequenceAsync(rightParking);
                            currentState = State.IDLE;
                        }
                        if(tagNumber == 3){
                            drive.followTrajectorySequenceAsync(leftParking);
                            currentState = State.IDLE;
                        }


                    }
                    break;


                case IDLE:


                    break;
                }
                drive.update();
                slide.fixSlides();
                telemetry.addData("tagID",tagNumber);
                telemetry.addData("state:",currentState);
                telemetry.update();
            }

        }

    }





