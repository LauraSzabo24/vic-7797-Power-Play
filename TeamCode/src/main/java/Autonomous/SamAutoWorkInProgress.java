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
public class SamAutoWorkInProgress extends LinearOpMode {
    //FSM states
    enum State {
        TO_POLE,   // First, follow a splineTo() trajectory
        TO_STACK,
        PARKING,// Then, we follow another lineTo() trajectory
        IDLE            // Our bot will enter the IDLE state when done
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
    public static double smallHeight = 2100;
    public static double midHeight =3141;
    public static double tallHeight =4175;
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

    int offSet = 0;
    int angleOffSet =0;
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
        Pose2d farmingPose = new Pose2d(-30.4,-6,Math.toRadians(45));
        Pose2d stackPose = new Pose2d(-59,-12,Math.toRadians(180));

        //Parking Coordinates

        Pose2d middlePark = new Pose2d(-35.8,-12,Math.toRadians(0));
        Pose2d leftPark =  new Pose2d(-57.8,-12,Math.toRadians(0));
        Pose2d rightPark =  new Pose2d(-10.8,-12,Math.toRadians(270));

        drive.setPoseEstimate(startPose);


        //trajectories and trajectory sequences


        drive.setPoseEstimate(startPose);

        TrajectorySequence firstCone = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    //bring up slides
                    //targetPosition = tallHeight;
                })
                .lineToLinearHeading(midPose)
                .splineToSplineHeading(farmingPose, Math.toRadians(45))
                .build();



        TrajectorySequence toStack = drive.trajectorySequenceBuilder(/*farming pose*/farmingPose)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{
                    //lower slides
                    //targetPosition = grabHeight;
                    //open claw
                    //openClaw();
                })
                .back(3)
                .splineToSplineHeading(/*stack pose*/stackPose, Math.toRadians(180))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{
                    //close claw lift
                    //closeClaw();
                    //targetPosition = tallHeight;
                    //offSet += 10;
                    //angleOffSet += 4;
                })
                .build();


        TrajectorySequence backToPole = drive.trajectorySequenceBuilder(toStack.end())
                .setReversed(true)
                .splineToSplineHeading(/*farming pose*/ farmingPose, Math.toRadians(60))
                .setReversed(false)
                .build();



        TrajectorySequence zone1 = drive.trajectorySequenceBuilder(backToPole.end())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{
                    //lower slides
                    //targetPosition = 0;
                    //open claw
                    //openClaw();
                })
                .setReversed(true)
                .back(3)
                .splineToSplineHeading(leftPark, Math.toRadians(180))
                .setReversed(false)
                .build();

        TrajectorySequence zone2 = drive.trajectorySequenceBuilder(backToPole.end())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{
                    //lower slides
                    //targetPosition = 0;
                    //open claw
                    //openClaw();
                })
                .lineToLinearHeading(new Pose2d(-35.4,-11,Math.toRadians(42)))
                .lineToLinearHeading(middlePark)
                .build();

        TrajectorySequence zone3 = drive.trajectorySequenceBuilder(backToPole.end())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{
                    //lower slides
                    //targetPosition = 0;
                    //open claw
                    //openClaw();
                })
                .setReversed(true)
                .back(3)
                .splineToSplineHeading(rightPark, Math.toRadians(0))
                .setReversed(false)
                .build();



        State currentState = State.TO_POLE;
        drive.followTrajectorySequenceAsync(firstCone);

        int i = 0;
        while(opModeIsActive())
        {
            switch (currentState) {
                case TO_POLE:
                    if (!drive.isBusy()) {
                        if(i<3) {
                            drive.followTrajectorySequenceAsync(toStack);
                            currentState = State.TO_STACK;
                            i++;
                        }
                        else{
                            switch (tagNumber) {
                                case 1 :
                                    drive.followTrajectorySequence(zone1);
                                    currentState = State.IDLE;
                                    break;
                                case 2 :
                                    drive.followTrajectorySequence(zone2);
                                    currentState = State.IDLE;
                                    break;
                                case 3 :
                                    drive.followTrajectorySequence(zone3);
                                    currentState = State.IDLE;
                                    break;
                            }
                        }
                    }
                    break;
                case TO_STACK:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(backToPole);
                        currentState = State.TO_POLE;
                        grabHeight -= 200;
                    }
                    break;
                case IDLE:
                    break;
            }
            drive.update();
            fixSlides();
            telemetry.addData("tagID",tagNumber);
            telemetry.addData("state:",currentState);
            telemetry.update();
        }

    }

    public void fixSlides()
    {
        telemetry.addData("positionLL:", pulleyMotorL.getCurrentPosition());
        if (Math.abs(targetPosition - pulleyMotorL.getCurrentPosition()) > 12 && opModeIsActive()) //&& (4000>pulleyMotorL.getCurrentPosition()) && (-10<pulleyMotorL.getCurrentPosition()))
        {
                double power = returnPower(targetPosition, pulleyMotorL.getCurrentPosition());
                //purely experimental
                pulleyMotorL.setPower(2*power);
                pulleyMotorR.setPower(2*power);
                telemetry.addData("positionLL:", pulleyMotorL.getCurrentPosition());
                telemetry.update();
        }
        else{
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
