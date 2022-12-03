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
public class GoodOneConePark extends LinearOpMode {

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
                .forward(38)
                .waitSeconds(0.5)
                .strafeRight(48)
                .waitSeconds(0.5)
                .build();
        TrajectorySequence dropCone = drive.trajectorySequenceBuilder(goToPole.end())
                .waitSeconds(0.5)
                .forward(6)
                .waitSeconds(0.5)
                .build();
        TrajectorySequence backwards = drive.trajectorySequenceBuilder(dropCone.end())
                .waitSeconds(0.5)
                .back(5)
                .waitSeconds(0.5)
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(backwards.end())
                .waitSeconds(0.4)
                .strafeLeft(12)
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(backwards.end())
                .waitSeconds(0.4)
                .strafeLeft(36)
                .build();
        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(backwards.end())
                .waitSeconds(0.4)
                .strafeLeft(60)
                .build();
        TrajectorySequence Timer = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .build();
        TrajectorySequence Timer2 = drive.trajectorySequenceBuilder(dropCone.end())
                .waitSeconds(3)
                .build();
        TrajectorySequence Timer3 = drive.trajectorySequenceBuilder(backwards.end())
                .waitSeconds(1)
                .build();
        waitForStart(); //also new

        while (opModeIsActive()&&stop<1) {
            /*targetPosition = 4115;
            double power = 0;
            telemetry.addData("positionLL:", pulleyMotorL.getCurrentPosition());
            while (Math.abs(targetPosition - pulleyMotorL.getCurrentPosition()) > 12) {
                power = returnPower(targetPosition, pulleyMotorL.getCurrentPosition());
                pulleyMotorL.setPower(power);
                pulleyMotorR.setPower(power);
                telemetry.addData("positionLL:", pulleyMotorL.getCurrentPosition());
                telemetry.update();
            }*/

            rightServo.setPosition(0.5);
            leftServo.setPosition(0.5);
            drive.followTrajectorySequence(Timer);

            //go to the tallest pole
            drive.followTrajectorySequence(goToPole);

            //PID slide moving to drop cone
            targetPosition = 4000;
            fixSlides();

            //go forward and open claw
            drive.followTrajectorySequence(dropCone);

            //bring the slides lower
            targetPosition = 3800;
            fixSlides();
            //hello
            drive.followTrajectorySequence(Timer2);
            rightServo.setPosition(0.2);
            leftServo.setPosition(0.8);
            drive.followTrajectorySequence(Timer2);

            /*double time = timer2.time();
            while (time < (time + 0.5)) {
                time = timer2.time();
            } //timer to put down the slides*/

            targetPosition = 2100;
            fixSlides();

            drive.followTrajectorySequence(backwards);


            //go back and lower slides
            //drive.followTrajectorySequence(backwards);
            targetPosition = 0;
            fixSlides();

            //close the claw
            rightServo.setPosition(0.5);
            leftServo.setPosition(0.5);
            drive.followTrajectorySequence(Timer3);

            //tag parking
            if (tagNumber == 1) {
                //if (!isStopRequested())
                drive.followTrajectorySequence(parkLeft);
                tagNumber = 4;
            } else if (tagNumber == 2) {
                //if (!isStopRequested())
                drive.followTrajectorySequence(centerPark);
                tagNumber = 4;
            } else if (tagNumber == 3) {
                //if (!isStopRequested())
                drive.followTrajectorySequence(parkRight);
                tagNumber = 4;
            }
            stop++;
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
        while (Math.abs(targetPosition - pulleyMotorL.getCurrentPosition()) > 12 && opModeIsActive())
        {
            power = returnPower(targetPosition, pulleyMotorL.getCurrentPosition());
            pulleyMotorL.setPower(power);
            pulleyMotorR.setPower(power);
            telemetry.addData("positionLL:", pulleyMotorL.getCurrentPosition());
            telemetry.update();
        }
    }

}
