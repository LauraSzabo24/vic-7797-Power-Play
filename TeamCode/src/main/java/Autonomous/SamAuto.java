package Autonomous;

//PID

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
public class SamAuto extends LinearOpMode {

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
    public static double smallHeight = 2100;
    public static double midHeight =3141;
    public static double tallHeight =4115;
    public static double grabHeight =940;
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

        telemetry.setMsTransmissionInterval(50);
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //Scoring Coordinates
        Pose2d startPose = new Pose2d(-35, -65, Math.toRadians(90));
        Pose2d midPose = new Pose2d(-34.5, -20, Math.toRadians(90));
        Pose2d approachPose = new Pose2d(-30.4,-6,Math.toRadians(45));
        //Parking Coordinates

        Pose2d middlePark = new Pose2d(-35.8,-34.6,Math.toRadians(270));
        Pose2d leftPark =  new Pose2d(-60.8,-35.6,Math.toRadians(270));
        Pose2d rightPark =  new Pose2d(-10.8,-35.6,Math.toRadians(270));

        drive.setPoseEstimate(startPose);


        //trajectories and trajectory sequencess


        drive.setPoseEstimate(startPose);

        TrajectorySequence firstCone = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    //bring up slides
                })
                .lineToLinearHeading(midPose)
                .splineToSplineHeading(approachPose, Math.toRadians(45))
                .build();



        TrajectorySequence toStack = drive.trajectorySequenceBuilder(firstCone.end())
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{
                    //lower slides interval
                    //open claw
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    //bring down slides-interval
                })
                .back(3)
                .splineToLinearHeading(new Pose2d(-36.4,-18,Math.toRadians(130)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-59,-12,Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{
                    //close claw lift
                })
                .build();


        Trajectory backToPole1 = drive.trajectoryBuilder(toStack.end(), true)
                .lineToLinearHeading(new Pose2d(-49,-12,Math.toRadians(180)))
                .build();


        Trajectory backToPole2 = drive.trajectoryBuilder(backToPole1.end(), true)
                .splineToLinearHeading(new Pose2d(-30.4,-6,Math.toRadians(45)), Math.toRadians(45))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder((backToPole2.end()))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{
                    //lower slides interval
                    //open claw
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    //bring down slides-interval
                })
                .lineToLinearHeading(new Pose2d(-35.4,-11,Math.toRadians(42)))
                .lineToLinearHeading(new Pose2d(-35,-15,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-35,-35,Math.toRadians(90)))
                .build();


        TrajectorySequence zone1 = drive.trajectorySequenceBuilder(park.end())
                .lineToLinearHeading(leftPark)
                        .build();

        TrajectorySequence zone2 = drive.trajectorySequenceBuilder(park.end())
                .lineToLinearHeading(middlePark)
                .build();

        TrajectorySequence zone3 = drive.trajectorySequenceBuilder(park.end())
                .lineToLinearHeading(rightPark)
                .build();





        closeClaw();
        drive.followTrajectorySequenceAsync(firstCone);
        for (int i = 5; i >= 2; i-- ) {
            drive.followTrajectorySequenceAsync(toStack);
            drive.followTrajectoryAsync(backToPole1);
            drive.followTrajectoryAsync(backToPole2);
        }
        drive.followTrajectorySequenceAsync(park);
        switch (tagNumber) {
            case 1 : drive.followTrajectorySequence(zone1);
            break;
            case 2 : drive.followTrajectorySequence(zone2);
            break;
            case 3 : drive.followTrajectorySequence(zone3);
            break;
        }

        while(opModeIsActive())
        {
            fixSlides();
            if(liftIsBusy){
                drive.setMotorPowers(0,0,0,0);
            }
            drive.update();


        }

    }

    public void fixSlides()
    {


        telemetry.addData("positionLL:", pulleyMotorL.getCurrentPosition());
        while (Math.abs(targetPosition - pulleyMotorL.getCurrentPosition()) > 12 && opModeIsActive()) //&& (4000>pulleyMotorL.getCurrentPosition()) && (-10<pulleyMotorL.getCurrentPosition()))
        {


                double power = returnPower(targetPosition, pulleyMotorL.getCurrentPosition());
                pulleyMotorL.setPower(power);
                pulleyMotorR.setPower(power);
                telemetry.addData("positionLL:", pulleyMotorL.getCurrentPosition());
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
