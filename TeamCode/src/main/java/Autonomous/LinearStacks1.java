package Autonomous;

//PID

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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
@Disabled
public class LinearStacks1 extends LinearOpMode
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
    public static double smallHeight = 2100;
    public static double midHeight =3141;
    public static double tallHeight =4115;
    public static double grabHeight =940;
    public static double targetPosition = 5;
    public static boolean targetIndicator = false;


    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    //PID ends here

    //CLAW goes here
    private Servo rightServo;
    private Servo leftServo;
    //CLAW ends here


    //added
    public static int tagNumber;
    public static int stackNum;
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
    public void runOpMode() {
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
            numberDetected = LinearStacks1.tagNumber;
        }

        //edited from here
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);




        //roadrunner trajectory stuff
        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(90));
        Pose2d stackPose = new Pose2d(-69.5,7.7,Math.toRadians(180));
        Pose2d farmPose = new Pose2d(-20.5,9,Math.toRadians(90));
        Pose2d midTravelPose = new Pose2d(-35,7,Math.toRadians(90));

        drive.setPoseEstimate(startPose);




        //trajectories and trajectory sequences
        TrajectorySequence startOff = drive.trajectorySequenceBuilder(startPose)

                .lineToLinearHeading(midTravelPose)

                .UNSTABLE_addTemporalMarkerOffset(-4.5,()->{
                    //bring up slides-interval
                    closeClaw();
                    targetPosition = smallHeight;
                })

                    .lineToLinearHeading(farmPose)
                    .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(-2,()->{
                    //bring up slides full
                    targetPosition = tallHeight;
                })

                .forward(8)
                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
                    //bring slides down-partial
                    //drop cone-release servo
                    targetPosition = tallHeight - 150;
                    openClaw();
                })

                .back(6)

                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{
                    //drop slides all the way
                    targetPosition = grabHeight;
                })

               .lineToLinearHeading(stackPose)

                .UNSTABLE_addTemporalMarkerOffset(-4,()-> {
                    closeClaw();
                    telemetry.addData("hello friends",pulleyMotorL.getCurrentPosition());
                })

                .build();






        TrajectorySequence scoreOnStack = drive.trajectorySequenceBuilder(startOff.end())
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(.5,()-> {

                    targetPosition = smallHeight;
                    fixSlides();
                })
               /* .lineToLinearHeading(farmPose)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-2,()->{
                    //bring up slides full
                    targetPosition = tallHeight;
                    fixSlides();
                })
                .forward(8)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    //bring slides down-partial
                    //drop cone-release servo
                    targetPosition = tallHeight - 250;
                    fixSlides();
                    openClaw();
                })
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{
                    //drop slides all the way
                    targetPosition = 0;
                    fixSlides();
                })
                .lineToLinearHeading(stackPose)
                .build();
        TrajectorySequence scoreOnStack2 = drive.trajectorySequenceBuilder(scoreOnStack.end())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-3,()-> {
                    //
                    targetPosition = 940;//shift slides
                    fixSlides();
                    openClaw();
                })
                .lineToLinearHeading(farmPose)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-.5,()->{
                    //bring up slides full
                    targetPosition = tallHeight;
                    fixSlides();
                })
                .forward(8)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    //bring slides down-partial
                    //drop cone-release servo
                    targetPosition = tallHeight - 250;
                    fixSlides();
                    openClaw();
                })
                .back(6)

                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{
                    //drop slides all the way
                    targetPosition = 0;
                    fixSlides();
                })
                .lineToLinearHeading(stackPose)
              */  .build();




        waitForStart(); //also new

        drive.followTrajectorySequence(startOff);
        drive.followTrajectorySequence(scoreOnStack);
       // drive.followTrajectorySequence(scoreOnStack2);


        if(numberDetected == 1){
            // park in zone 1
            telemetry.addData("PARK IN ZONE 1", numberDetected);
            // drive.followTrajectorySequence(parkLeft);
            tagNumber=4;
        }
        else if(numberDetected == 2) {
            // park in zone 2
            telemetry.addData("PARK IN ZONE 2", numberDetected);
            // drive.followTrajectory(centerPark);
            tagNumber=4;
        }
        else if(numberDetected ==3) {
            // park in zone 3
            telemetry.addData("PARK IN ZONE 3", numberDetected);
            // drive.followTrajectorySequence(parkRight);
            tagNumber=4;
        }
        while (opModeIsActive()) {

            fixSlides();
        }
    }
    public void fixSlides() {
        while(Math.abs((targetPosition-pulleyMotorL.getCurrentPosition()))>15) {
            double power = returnPower(targetPosition, pulleyMotorL.getCurrentPosition());
            pulleyMotorL.setPower(power);
            pulleyMotorR.setPower(power);
            telemetry.addData("position", pulleyMotorL.getCurrentPosition());
            telemetry.addData("pwoer", power);
            telemetry.update();

        }
        pulleyMotorL.setPower(0);
        pulleyMotorR.setPower(0);
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

