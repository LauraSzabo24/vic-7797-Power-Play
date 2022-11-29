package Autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

//PID
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

//Servo
import com.qualcomm.robotcore.hardware.Servo;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import pipelines.AprilTagDetectionPipeline;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class Park extends LinearOpMode
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
    public void runOpMode() throws InterruptedException
    {
        initialize();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //roadrunner trajectory stuff
        Pose2d startPose = new Pose2d(0,0,0);
        //drive.setPoseEstimate(startPose);

        //trajectories and trajectory sequences

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(startPose)
                .forward(35)
                .waitSeconds(1)
                .strafeLeft(24.0)
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(startPose)
                .forward(35)
                .waitSeconds(1)
                .strafeRight(24)
                .build();
        Trajectory centerPark  = drive.trajectoryBuilder(startPose)
                .forward(35)
                .build();

        waitForStart(); //also new

        //from here2
        while (opModeIsActive())
        {
            if (tagNumber == 1)
            {
                //if (!isStopRequested())
                    drive.followTrajectorySequence(parkLeft);
            }
            else if (tagNumber == 2)
            {
                //if (!isStopRequested())
                    drive.followTrajectory(centerPark);
            }
            else if (tagNumber == 3)
            {
                //if (!isStopRequested())
                    drive.followTrajectorySequence(parkRight);
            }
        }

    }

}

