package Autonomous;

//PID

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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


@Config
@Autonomous
public class StationaryAuto extends LinearOpMode {
    enum State {
        FIRST_CONE,
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
    public static double midHeight =3141;
    public static double tallHeight =3950;
    public static double grabHeight =800; //940
    public static boolean liftIsBusy = false;
    public static double targetPosition = 0;


    public static double aPx = -37.1;
    public static double aPy = -9.7;//i am so funny & indra is dumb fr

    public static double fPx = -30.2;//-30.2
    public static double fPy = -4.4;//-3.5

    public static double sPx = -63.5;
    public static double sPy = -8.1; //-8.1

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
        PIDFController pidSlide = new PIDFController(Kp,Ki,Kd,Kf);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //Scoring Coordinates




        Pose2d approachPose = new Pose2d(aPx, aPy, Math.toRadians(45));//heading orgin:47
        Pose2d startPose = new Pose2d(-36, -62, Math.toRadians(90));
        Pose2d farmPose = new Pose2d(fPx,fPy,Math.toRadians(45));
        Pose2d stackPose = new Pose2d(sPx,sPy,Math.toRadians(0));


        Pose2d middlePark = new Pose2d(-35,-8.1,Math.toRadians(90));
        Pose2d leftPark =  new Pose2d(-60.8,-8.1,Math.toRadians(90));
        Pose2d rightPark =  new Pose2d(-11.8,-8.1,Math.toRadians(90));

        drive.setPoseEstimate(startPose);


        //trajectories and trajectory sequences


        drive.setPoseEstimate(startPose);

        TrajectorySequence FirstCone = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(aPx, aPy, Math.toRadians(90)))
                .turn(Math.toRadians(-45))
                .lineToLinearHeading(farmPose) //make exactly on pole
                .waitSeconds(0.5)
                .lineToLinearHeading(approachPose)
                .build();


        TrajectorySequence ToStack = drive.trajectorySequenceBuilder(FirstCone.end())// or farmpose
                .turn(Math.toRadians(135))
                .lineToLinearHeading(stackPose)
                .waitSeconds(0.7)
                .lineToLinearHeading(new Pose2d(aPx, aPy, Math.toRadians(180)))// or change to approach pose if needed
                .build();

        TrajectorySequence ToPole = drive.trajectorySequenceBuilder(ToStack.end())//.plus(new Pose2d(-2,-4,0))
                .turn(Math.toRadians(-135))
                .lineToLinearHeading(farmPose) //make this exactly on the pole new Pose2d(fPx,fPy,Math.toRadians(50))
                .waitSeconds(0.5)
                .lineToLinearHeading(approachPose)
                .build();



        TrajectorySequence zone1 = drive.trajectorySequenceBuilder(ToPole.end())
                .lineToLinearHeading(leftPark)
                .build();

        TrajectorySequence zone2 = drive.trajectorySequenceBuilder(ToPole.end())
                .lineToLinearHeading(middlePark)
                .build();

        TrajectorySequence zone3 = drive.trajectorySequenceBuilder(ToPole.end())
                .lineToLinearHeading(rightPark)

                .build();






        closeClaw();
        cycle =0;
        grabHeight =800;

        drive.followTrajectorySequenceAsync(FirstCone);
        ElapsedTime et = new ElapsedTime(0);
        State currentState = State.FIRST_CONE;


        while(opModeIsActive())
        {
            switch (currentState) {
                case FIRST_CONE:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(ToStack);
                        currentState = State.TO_STACK;
                        et.reset();
                        cycle++;
                    }
                    if (et.seconds() == 0) targetPosition = tallHeight;
                    else if (et.seconds() == 2) {
                        targetPosition -= 200;
                        openClaw();
                    }
                    else if(et.seconds() == 3) {
                        targetPosition = grabHeight;
                    }
                case TO_POLE:
                    if (!drive.isBusy()) {
                        if(cycle<2) {
                            drive.followTrajectorySequenceAsync(ToStack);
                            currentState = State.TO_STACK;
                            et.reset();
                            cycle++;
                        }
                        else {
                            switch (tagNumber) {
                                case 1 :
                                    closeClaw();
                                    targetPosition = 50;
                                    drive.followTrajectorySequenceAsync(zone1);
                                    currentState = State.IDLE;
                                    break;
                                case 2 :
                                    closeClaw();
                                    targetPosition = 50;
                                    drive.followTrajectorySequenceAsync(zone2);
                                    currentState = State.IDLE;
                                    break;
                                case 3 :
                                    closeClaw();
                                    targetPosition = 50;
                                    drive.followTrajectorySequenceAsync(zone3);
                                    currentState = State.IDLE;
                                    break;
                            }
                        }
                        if (et.seconds() == 0) targetPosition = tallHeight;
                        else if (et.seconds() == 2) {
                            targetPosition -= 200;
                            openClaw();
                        }
                        else if(et.seconds() == 3.1) {
                            targetPosition = grabHeight;
                        }
                    }
                    break;
                case TO_STACK:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(ToPole);
                        currentState = State.TO_POLE;
                        et.reset();
                        grabHeight -= 200;
                    }
                    if (et.seconds() == 2) closeClaw();
                    else if (et.seconds() == 2.5) {
                        targetPosition = tallHeight;
                    }
                    break;
                case IDLE:
                    break;


            }
            drive.update();
            fixSlides();
            telemetry.addData("offset",fPy);
            telemetry.addData("timer",et.seconds());
            telemetry.update();

        }



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
