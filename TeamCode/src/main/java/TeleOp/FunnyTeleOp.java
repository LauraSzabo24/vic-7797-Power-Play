package TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;



@TeleOp(name = "FunnyTeleOp")
public class FunnyTeleOp extends OpMode {

    //mecanum constants
    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;

    private double getMax(double[] input) {
        double max = Integer.MIN_VALUE;
        for (double value : input) {
            if (Math.abs(value) > max) {
                max = Math.abs(value);
            }
        }
        return max;
    }

    enum State  {
            TRAJ2, TRAJ3
    }

    //servo constants
    private Servo rightServo;
    private Servo leftServo;


    //slide constants
    private Motor liftA;
    private Motor liftB;


    private double targetPos;


    //PID slides constants
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
    public static double tallHeight =4215;
    public static double motorPower =1.3;
    public static double targetPosition = 5;

   // Thread slideUpdate = new Thread(new SlideFixerT());
   // SlideFixerT slideSet = new SlideFixerT();
    public static boolean OpmodeAvil;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    Pose2d terminalPose = new Pose2d(0, -55, Math.toRadians(270));
    Pose2d startPose = new Pose2d(-10.8,-35.6,Math.toRadians(270));
    Pose2d highPose = new Pose2d(0,-32,Math.toRadians(90));
    Pose2d midPose = new Pose2d(0,-35.6,Math.toRadians(270));
    Pose2d midRightPose = new Pose2d(17,-30,Math.toRadians(45));
    Pose2d midLeftPose = new Pose2d(-17,-30,Math.toRadians(135));

    TrajectorySequence stuff2;
    TrajectorySequence stuff3;


    @Override
    public void init() {
        //motor initialization
      //  slideUpdate.start();
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("BR");

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //servo initialization

        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");



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

        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        drive.setPoseEstimate(startPose);


        //trajectories and trajectory sequences


        drive.setPoseEstimate(startPose);

        stuff2 = drive.trajectorySequenceBuilder(startPose)
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

        stuff3 = drive.trajectorySequenceBuilder(stuff2.end())
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

    }

    @Override
    public void loop() {
        // mecanum
        boolean precisionToggle = gamepad1.right_trigger > 0.1;
        drive(precisionToggle);
        OpmodeAvil = true;

        //pivots
        if(gamepad1.b)
        {
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(0.9);
            motorFrontRight.setPower(-0.9);

        }
        if(gamepad1.a)
        {
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(-0.9);
            motorFrontRight.setPower(0.9);


        }

        //servos
        if(gamepad2.b)
        {
            rightServo.setPosition(0.5);
            leftServo.setPosition(0.5);
        }
        if(gamepad2.a)
        {
            rightServo.setPosition(0.2);
            leftServo.setPosition(0.8);
        }



        //PID
        if(gamepad2.dpad_up && pulleyMotorL.getCurrentPosition()<5000)
        {
            targetPosition = tallHeight;
          //  slideSet.setHeight(tallHeight);

        }
        if(gamepad2.dpad_left && pulleyMotorL.getCurrentPosition()>50)
        {
            targetPosition = smallHeight;
          //  slideSet.setHeight(smallHeight);

        }
        if(gamepad2.dpad_right)
        {
            targetPosition = midHeight;
          //  slideSet.setHeight(midHeight);


        }
        if(gamepad2.dpad_down)
        {
            targetPosition = 0;
          //  slideSet.setHeight(0);
        }

      if(gamepad2.right_bumper && pulleyMotorL.getCurrentPosition()<5000)
        {

            pulleyMotorL.setPower(motorPower);
            pulleyMotorR.setPower(motorPower);
           targetPosition = pulleyMotorL.getCurrentPosition();



        }
        if(gamepad2.left_bumper && pulleyMotorL.getCurrentPosition() > 0)
        {
            pulleyMotorL.setPower(-motorPower);
            pulleyMotorR.setPower(-motorPower);
            targetPosition = pulleyMotorL.getCurrentPosition();



        }
        if(!gamepad2.right_bumper && !gamepad2.left_bumper && Math.abs(targetPosition-pulleyMotorL.getCurrentPosition())<12)
        {

            pulleyMotorL.setPower(0);
            pulleyMotorR.setPower(0);

        }

        State currentState = State.TRAJ2;
        if(gamepad1.dpad_down) {
            while (!gamepad1.dpad_up) {
                switch (currentState) {
                    case TRAJ2:
                        if (!drive.isBusy()) {
                            drive.followTrajectorySequenceAsync(stuff2);
                            currentState = State.TRAJ3;
                        }
                        break;

                    case TRAJ3:
                        if (!drive.isBusy()) {
                            drive.followTrajectorySequenceAsync(stuff3);
                        }
                        break;
                }
                drive.update();
                fixSlides();
            }
        }


        double power = returnPower(targetPosition, pulleyMotorL.getCurrentPosition());
        telemetry.addData("positonrightMotor", pulleyMotorR.getCurrentPosition());
        telemetry.addData("positonleftMotor", pulleyMotorL.getCurrentPosition());
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("power", power);
        pulleyMotorL.setPower(power);
        pulleyMotorR.setPower(power);
        telemetry.update();


    }



    public void stop(){
        OpmodeAvil = false;

    }
    //mecanum methods
    public void drive(boolean precisionToggle) {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        boolean superToggle = gamepad1.left_trigger > 0.1;


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Calculate the mecanum motor powers
        double frontLeftPower = (y + x + 2 * rx) / denominator;
        double backLeftPower = (y - x + 2 * rx) / denominator;
        double frontRightPower = (y - x - 2 * rx) / denominator;
        double backRightPower = (y + x - 2 * rx) / denominator;


        // Cube the motor powers
        frontLeftPower = Math.pow(frontLeftPower, 3);
        frontRightPower = Math.pow(frontRightPower, 3);
        backLeftPower = Math.pow(backLeftPower, 3);
        backRightPower = Math.pow(backRightPower, 3);

        // Calculate the maximum value of all the motor powers
        // The argument here is just an array separated into different lines
        double maxValue = getMax(new double[]{
                frontLeftPower,
                frontRightPower,
                backLeftPower,
                backRightPower
        });

        // Resize the motor power values
        if (maxValue > 1) {
            frontLeftPower /= maxValue;
            frontRightPower /= maxValue;
            backLeftPower /= maxValue;
            backRightPower /= maxValue;
        }


        if(gamepad1.right_bumper){
            frontLeftPower = (   x + 2 * rx) / denominator;
            backLeftPower = (  - x + 2 * rx) / denominator;
            frontRightPower = ( - x - 2 * rx) / denominator;
            backRightPower = (  x - 2 * rx) / denominator;

        }

        if (superToggle) {
            motorFrontLeft.setPower(frontLeftPower * 0.2);
            motorBackLeft.setPower(backLeftPower * 0.2);
            motorFrontRight.setPower(frontRightPower * 0.2);
            motorBackRight.setPower(backRightPower * 0.2);
        }
        if (precisionToggle) {
            motorFrontLeft.setPower(frontLeftPower * 0.4);
            motorBackLeft.setPower(backLeftPower * 0.4);
            motorFrontRight.setPower(frontRightPower * 0.4);
            motorBackRight.setPower(backRightPower * 0.4);
        }


        else
        {
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);


        }

    }

    //PID methods
    public double returnPower(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error -lastError)/ timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output; //figure out how to connect dashboard

    }

    public void fixSlides()
    {


        if (Math.abs(targetPosition - pulleyMotorL.getCurrentPosition()) > 12) //&& (4000>pulleyMotorL.getCurrentPosition()) && (-10<pulleyMotorL.getCurrentPosition()))
        {


            double power = returnPower(targetPosition, pulleyMotorL.getCurrentPosition());
            pulleyMotorL.setPower(power);
            pulleyMotorR.setPower(power);
            telemetry.addData("positonrightMotor", pulleyMotorR.getCurrentPosition());
            telemetry.addData("positonleftMotor", pulleyMotorL.getCurrentPosition());
            telemetry.addData("targetPosition", targetPosition);
            telemetry.addData("power", power);
            telemetry.update();




        }
        pulleyMotorL.setPower(0);
        pulleyMotorR.setPower(0);

    }

}
