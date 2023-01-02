package Autonomous;

//PID

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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


@Autonomous
@Disabled
public class FunnyTeleOpHelper extends LinearOpMode {

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
        //Trajectory starts here
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //Scoring Coordinates
        Pose2d terminalPose = new Pose2d(0, -55, Math.toRadians(270));
        Pose2d startPose = new Pose2d(-10.8,-35.6,Math.toRadians(270));
        Pose2d highPose = new Pose2d(0,-32,Math.toRadians(90));
        Pose2d midPose = new Pose2d(0,-35.6,Math.toRadians(270));
        Pose2d midRightPose = new Pose2d(17,-30,Math.toRadians(45));
        Pose2d midLeftPose = new Pose2d(-17,-30,Math.toRadians(135));
        //Parking Coordinates
        drive.setPoseEstimate(startPose);


        //trajectories and trajectory sequences


        drive.setPoseEstimate(startPose);



        TrajectorySequence stuff2 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(midPose)
                .lineToLinearHeading(terminalPose)
                .setReversed(true)
                .back(8)
                .lineToSplineHeading(highPose)
                .setReversed(false)
                .waitSeconds(0.5)
                .lineToSplineHeading(terminalPose)
                .waitSeconds(0.5)

                .setReversed(true)
                .lineToLinearHeading(new Pose2d(0,-49,Math.toRadians(270)))
                .splineToSplineHeading(midRightPose,Math.toRadians(45))
                .setReversed(false)
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(0,-49,Math.toRadians(270)),Math.toRadians(270))
                .lineToLinearHeading(terminalPose)
                .setReversed(false)
                .waitSeconds(0.5)

                .setReversed(true)
                .lineToLinearHeading(new Pose2d(0,-49,Math.toRadians(270)))
                .splineToSplineHeading(midLeftPose,Math.toRadians(135))
                .setReversed(false)
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(0,-49,Math.toRadians(270)),Math.toRadians(270))
                .lineToLinearHeading(terminalPose)
                .setReversed(false)
                .waitSeconds(0.5)
                .build();

        TrajectorySequence stuff3 = drive.trajectorySequenceBuilder(stuff2.end())
                .setReversed(true)
                .back(8)
                .lineToSplineHeading(highPose)
                .setReversed(false)
                .waitSeconds(0.5)
                .lineToSplineHeading(terminalPose)
                .waitSeconds(0.5)

                .setReversed(true)
                .lineToLinearHeading(new Pose2d(0,-49,Math.toRadians(270)))
                .splineToSplineHeading(midRightPose,Math.toRadians(45))
                .setReversed(false)
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(0,-49,Math.toRadians(270)),Math.toRadians(270))
                .lineToLinearHeading(terminalPose)
                .setReversed(false)
                .waitSeconds(0.5)

                .setReversed(true)
                .lineToLinearHeading(new Pose2d(0,-49,Math.toRadians(270)))
                .splineToSplineHeading(midLeftPose,Math.toRadians(135))
                .setReversed(false)
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(0,-49,Math.toRadians(270)),Math.toRadians(270))
                .lineToLinearHeading(terminalPose)
                .setReversed(false)
                .waitSeconds(0.5)
                .build();







        closeClaw();
        drive.followTrajectorySequenceAsync(stuff2);
        while (drive.isBusy()) {
            if (!gamepad2.y) {
                drive.update();
                fixSlides();
            }
            else return;
        }
        while (opModeInInit()) {
            drive.followTrajectorySequenceAsync(stuff3);
            while (drive.isBusy()) {
                if(!gamepad2.y) {
                    drive.update();
                    fixSlides();
                }
                else return;
            }
        }



    }


    public void fixSlides()
    {


        telemetry.addData("positionLL:", pulleyMotorL.getCurrentPosition());
        if (Math.abs(targetPosition - pulleyMotorL.getCurrentPosition()) > 12) //&& (4000>pulleyMotorL.getCurrentPosition()) && (-10<pulleyMotorL.getCurrentPosition()))
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
