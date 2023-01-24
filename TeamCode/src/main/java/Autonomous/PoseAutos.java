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
public class PoseAutos extends LinearOpMode {
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
    ElapsedTime timer = new ElapsedTime();

    //stop

    private double lastError = 0;
    private double integralSum = 0;

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

        try {
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
        catch (Exception e) {
            telemetry.addData("PreLoopStatus", "PRELOOP CAMERA ERROR, EXCEPTION CAUGHT");
            telemetry.update();
        }
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
        //Trajectory starts here
        PIDFController pidSlide = new PIDFController(Kp,Ki,Kd,Kf);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //Scoring Coordinates




        Pose2d approachPose = new Pose2d(aPx-1, aPy-8, Math.toRadians(57));//heading orgin:57
        Pose2d startPose = new Pose2d(-36, -62, Math.toRadians(90));
        Pose2d farmPose = new Pose2d(fPx,fPy,Math.toRadians(57));
        Pose2d stackPose = new Pose2d(sPx,sPy,Math.toRadians(180));


        Pose2d middlePark = new Pose2d(-35,-6.7,Math.toRadians(57));
        Pose2d leftPark =  new Pose2d(-55.8,-6.7,Math.toRadians(57));
        Pose2d rightPark =  new Pose2d(-11.8,-6.7,Math.toRadians(57));

        drive.setPoseEstimate(startPose);


        //trajectories and trajectory sequences


        drive.setPoseEstimate(startPose);
        TrajectorySequence bigTrajectory = drive.trajectorySequenceBuilder(startPose)
                //FIRST CONE
                .lineToLinearHeading(new Pose2d(aPx-2, aPy+8, Math.toRadians(90)))
                //.lineToLinearHeading(approachPose)
                .lineToLinearHeading(new Pose2d(aPx-1, aPy-8, Math.toRadians(57)))
                .lineToLinearHeading(new Pose2d(-30.5,-4.5,Math.toRadians(47))) //y:-2.5, heading: 45

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                    targetPosition = tallHeight;
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    for(int i =0; i<130; i++) openClaw();
                    targetPosition = tallHeight-200;

                })
                //.lineToLinearHeading(approachPose)
                .lineToLinearHeading(new Pose2d(aPx-1, aPy-8, Math.toRadians(57)))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    drive.setPoseEstimate(approachPose);
                })

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    targetPosition = grabHeight;
                    // drive.setPoseEstimate(new Pose2d(aPx-1, aPy-2, Math.toRadians(57)));
                })

                //GRABSTACK1

                //.lineToLinearHeading(new Pose2d(-62.8,-9.7,Math.toRadians(180)))//-6.7
                .lineToLinearHeading(stackPose)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    //closeClaw();
                    closeClaw();

                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> { // offset:-0.5
                    targetPosition = tallHeight+100;
                })
                .lineToLinearHeading(approachPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { // offset:-0.5
                    drive.setPoseEstimate(approachPose);
                })


                //GOTOPOLE1
                .lineToLinearHeading(new Pose2d(-30.5,-4.5,Math.toRadians(47))) //y:-1.8, heading:51 make this exactly on the pole new Pose2d(fPx,fPy,Math.toRadians(50))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    for(int i =0; i<130; i++) openClaw();
                    targetPosition = tallHeight-200;

                })
                .lineToLinearHeading(approachPose)//add offset if wished
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { //original offset = -0.5
                    grabHeight -= 200;
                    targetPosition = grabHeight;
                    drive.setPoseEstimate(approachPose);
                    // drive.setPoseEstimate(new Pose2d(aPx-1, aPy-2, Math.toRadians(57)));
                })
                //GRABSTACK2

                .lineToLinearHeading(stackPose)//-11.7
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    //closeClaw();
                    closeClaw();

                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    targetPosition = tallHeight+100;
                })
                .lineToLinearHeading(approachPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { // offset:-0.5
                    drive.setPoseEstimate(approachPose);
                })
                //.lineToLinearHeading(new Pose2d(aPx-1, -5.7, Math.toRadians(57)))



                //GOTOPOLE2
                .lineToLinearHeading(new Pose2d(-30.5,-4.5,Math.toRadians(47))) //heading: 53  make this exactly on the pole new Pose2d(fPx,fPy,Math.toRadians(50))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    for(int i =0; i<130; i++) openClaw();
                    targetPosition = tallHeight-200;

                })
                .lineToLinearHeading(approachPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { //original offset = -0.5
                    grabHeight -= 200;
                    targetPosition = grabHeight;
                    drive.setPoseEstimate(approachPose);
                })


                //GRABSTACK3

                  .lineToLinearHeading(new Pose2d(-62.8,-3.7,Math.toRadians(180)))//y:-12
                  .waitSeconds(0.5)
                  .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                      //closeClaw();
                      closeClaw();

                  })
                  .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                      targetPosition = tallHeight+100;
                  })
                  .lineToLinearHeading(approachPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { // offset:-0.5
                    drive.setPoseEstimate(approachPose);
                })
                  //.lineToLinearHeading(new Pose2d(aPx-1, -5.7, Math.toRadians(57)))



                  //GOTOPOLE3
                  .lineToLinearHeading(new Pose2d(-30.5,-4.5,Math.toRadians(47)))//y: -0.5 heading: 59 make this exactly on the pole new Pose2d(fPx,fPy,Math.toRadians(50))
                  .waitSeconds(0.7)
                  .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                      for(int i =0; i<130; i++) openClaw();
                      targetPosition = tallHeight-200;


                  })
                  .lineToLinearHeading(approachPose)
                  .UNSTABLE_addTemporalMarkerOffset(0, () -> { //original offset = -0.5
                      grabHeight -= 200;
                      targetPosition = grabHeight;

                  })



                .build();






        TrajectorySequence zone1 = drive.trajectorySequenceBuilder(bigTrajectory.end())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{
                    closeClaw();
                    targetPosition = 50;
                })
                //.lineToLinearHeading(new Pose2d(-35.4,-11,Math.toRadians(42)))
                // .lineToLinearHeading(middlePark)
                .lineToLinearHeading(leftPark)
                .build();

        TrajectorySequence zone2 = drive.trajectorySequenceBuilder(bigTrajectory.end())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{
                    closeClaw();
                    targetPosition = 50;
                })
                // .lineToLinearHeading(new Pose2d(-35.4,-11,Math.toRadians(42)))
                .lineToLinearHeading(middlePark)
                .build();

        TrajectorySequence zone3 = drive.trajectorySequenceBuilder(bigTrajectory.end())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{
                    closeClaw();
                    targetPosition = 50;
                })
                // .lineToLinearHeading(new Pose2d(-35.4,-11,Math.toRadians(42)))
                // .lineToLinearHeading(middlePark)
                .lineToLinearHeading(rightPark)
                .build();


        //waitForStart();

        telemetry.setMsTransmissionInterval(50);

        //from here2
        while (opModeInInit()) {
            try {
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
            catch (Exception e) {
                telemetry.addData("Status", "CAMERA ERROR, EXCEPTION CAUGHT");
                telemetry.update();
            }
        }



        //Trajectory starts here
        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //Scoring Coordinates







        cycle =0;
        grabHeight =700;
       // closeClaw();

        drive.followTrajectorySequenceAsync(bigTrajectory);
        State currentState = State.TO_POLE;

        while(opModeIsActive())
        {
            switch (currentState) {
                case TO_POLE:
                    if (!drive.isBusy()) {
                            drive.setPoseEstimate(approachPose);
                            switch (tagNumber) {
                                case 1 :
                                    drive.followTrajectorySequenceAsync(zone1);
                                    currentState = State.IDLE;
                                    break;
                                case 2 :
                                    drive.followTrajectorySequenceAsync(zone2);
                                    currentState = State.IDLE;
                                    break;
                                case 3 :
                                    drive.followTrajectorySequenceAsync(zone3);
                                    currentState = State.IDLE;
                                    break;
                                default :
                                   // drive.followTrajectorySequenceAsync(zone2);
                                    currentState = State.IDLE;
                                    break;

                        }
                    }
                    break;
                case IDLE:
                    break;


            }
            drive.update();
            fixSlides();//pp
            telemetry.addData("offset",fPy);
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
