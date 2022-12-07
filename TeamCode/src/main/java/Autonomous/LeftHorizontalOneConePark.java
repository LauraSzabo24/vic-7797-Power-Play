package Autonomous;

//PID

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
public class LeftHorizontalOneConePark extends LinearOpMode {

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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //roadrunner trajectory stuff
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence goToPole = drive.trajectorySequenceBuilder(startPose) //change this to go to front pole not side
               .addTemporalMarker(0, () -> {
                    rightServo.setPosition(0.5);
                    leftServo.setPosition(0.5);
                })
                .waitSeconds(0.5)
                .forward(38)
                .waitSeconds(0.2)
                .strafeRight(47)
                .waitSeconds(5)
                .addTemporalMarker(6, () -> {
                    targetPosition = 4100;
                    fixSlides();
                })
                .forward(4)
                .build();
        TrajectorySequence dropCone = drive.trajectorySequenceBuilder(goToPole.end())
                //.forward(4)
                .addTemporalMarker(0, () -> {
                    rightServo.setPosition(0.2);
                    leftServo.setPosition(0.8);

                })
                .waitSeconds(2)
                .back(6)
                .build();
        /*TrajectorySequence backwards = drive.trajectorySequenceBuilder(dropCone.end())
                .waitSeconds(0.5)
                .back(6)
                .build();*/
       TrajectorySequence slides = drive.trajectorySequenceBuilder(dropCone.end())
               .addTemporalMarker(0, () -> {
                    targetPosition = 0;
                    fixSlides();
                })
                .waitSeconds(3)
                .build();

        Pose2d poseEstimate = drive.getPoseEstimate();
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(dropCone.end())
                .strafeLeft(76)
                .waitSeconds(30)
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(dropCone.end())
                .strafeLeft(20)
                .waitSeconds(30)
                .build();
        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(dropCone.end())
                .strafeLeft(46)
                .waitSeconds(30)
                .build();

        while (opModeIsActive()&&stop<1) {
            //go to the tallest pole
            drive.followTrajectorySequence(dropCone);
            drive.followTrajectorySequence(goToPole);
            drive.followTrajectorySequence(dropCone);
            drive.followTrajectorySequence(slides);
            //tag parking
            if (tagNumber == 1) {
                //if (!isStopRequested())
                telemetry.addData("number1", tagNumber);
                drive.followTrajectorySequence(parkLeft);
                tagNumber = 4;
                stop++;
            } else if (tagNumber == 2) {
                //if (!isStopRequested())
                telemetry.addData("number2", tagNumber);
                drive.followTrajectorySequence(centerPark);
                tagNumber = 4;
                stop++;
            } else if (tagNumber == 3) {
                //if (!isStopRequested())
                telemetry.addData("number3", tagNumber);
                drive.followTrajectorySequence(parkRight);
                tagNumber = 4;
                stop++;
            }
        }
        while(opModeIsActive()&&stop>0)
        {
            targetPosition = 0;
            fixSlides();
        }
        while(opModeIsActive())
        {
            telemetry.addData("current position", pulleyMotorL.getCurrentPosition());
            telemetry.addData("target position", targetPosition);
            telemetry.addData("tag", tagNumber);
            telemetry.addData("current pose", new Pose2d(poseEstimate.getX(), poseEstimate.getY()));
            telemetry.addData("backwards.end()", dropCone.end());
            telemetry.update();
        }
    }

    public void fixSlides()
    {
        /*double power =0;
        while ((targetPosition - pulleyMotorL.getCurrentPosition()) > 12) {
               power = returnPower(targetPosition, pulleyMotorL.getCurrentPosition());
            pulleyMotorL.setPower(power);
            pulleyMotorR.setPower(power);
            targetPosition = pulleyMotorL.getCurrentPosition();
            telemetry.addData("positionLL:",pulleyMotorL.getCurrentPosition());
            }*/
        double power = 0;
        telemetry.addData("positionLL:", pulleyMotorL.getCurrentPosition());
        while (Math.abs(targetPosition - pulleyMotorL.getCurrentPosition()) > 12 && opModeIsActive()) //&& (4000>pulleyMotorL.getCurrentPosition()) && (-10<pulleyMotorL.getCurrentPosition()))
        {
            if(!(pulleyMotorL.getCurrentPosition()>4100))
            {
                power = returnPower(targetPosition, pulleyMotorL.getCurrentPosition());
                pulleyMotorL.setPower(power);
                pulleyMotorR.setPower(power);
                telemetry.addData("positionLL:", pulleyMotorL.getCurrentPosition());
                telemetry.update();
            }
        }
        pulleyMotorL.setPower(0);
        pulleyMotorR.setPower(0);
    }


}
