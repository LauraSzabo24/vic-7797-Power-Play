package TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "AntiTipTeleOp")
public class AntiTipTeleOp extends OpMode {

    //mecanum constants
    enum State{
        NORMAL,
        EMERGENCY
    }
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
    public static double motorPower =5;//1.3
    public static double targetPosition = 5;
    State currentState = State.NORMAL;

    // Thread slideUpdate = new Thread(new SlideFixerT());
   // SlideFixerT slideSet = new SlideFixerT();
    public static boolean OpmodeAvil;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    IMU imu;
    YawPitchRollAngles robotOrientation;
    double Yaw;
    double Pitch;
    double Roll;
    boolean isActivated = false;





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

        IMU.Parameters myIMUParameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
        );
        imu.initialize(myIMUParameters);
        robotOrientation = imu.getRobotYawPitchRollAngles();
        Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        Roll = robotOrientation.getRoll(AngleUnit.DEGREES);
        imu.resetYaw();

    }

    @Override
    public void loop() {
        // mecanum
        drive();
        OpmodeAvil = true;
        currentState = State.NORMAL;
        //pivots
        if(gamepad1.b)
        {
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(2);//0.9
            motorFrontRight.setPower(-2);

        }
        if(gamepad1.a)
        {
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(-2);
            motorFrontRight.setPower(2);


        }

        //servos
        if(gamepad2.b)
        {
            rightServo.setPosition(0.55);
            leftServo.setPosition(0.45);
        }
        if(gamepad2.a)
        {
            rightServo.setPosition(0.34);
            leftServo.setPosition(0.66);
        }


        if(gamepad2.b) {
            rightServo.setPosition(0.55);
            leftServo.setPosition(0.45);
        }
        if(gamepad2.a) {
            rightServo.setPosition(0.34);
            leftServo.setPosition(0.66);
        }

                //PID
        if(gamepad2.dpad_up) {
            targetPosition = tallHeight;
            //  slideSet.setHeight(tallHeight);

        }
        if(gamepad2.dpad_left) {
            targetPosition = smallHeight;
            //  slideSet.setHeight(smallHeight);

        }
        if(gamepad2.dpad_right) {
            targetPosition = midHeight;
            //  slideSet.setHeight(midHeight);

        }
        if(gamepad2.dpad_down) {
            targetPosition = 50;//180
            //  slideSet.setHeight(0);
        }

        if(gamepad2.right_bumper) {

            pulleyMotorL.setPower(0.08*motorPower);
            pulleyMotorR.setPower(0.08*motorPower);
            currentState = State.EMERGENCY;
            targetPosition = pulleyMotorL.getCurrentPosition();

        }
        if(gamepad2.left_bumper) {

            pulleyMotorL.setPower(-0.08*motorPower);
            pulleyMotorR.setPower(-0.08*motorPower);
            currentState = State.EMERGENCY;
            targetPosition = pulleyMotorL.getCurrentPosition();

        }
        if(!gamepad2.right_bumper && !gamepad2.left_bumper && Math.abs(targetPosition-pulleyMotorL.getCurrentPosition())<12) {

            pulleyMotorL.setPower(0);
            pulleyMotorR.setPower(0);

        }
        if(currentState== State.NORMAL){
            double power = returnPower(targetPosition, pulleyMotorL.getCurrentPosition());
            telemetry.addData("positonrightMotor", pulleyMotorR.getCurrentPosition());
            telemetry.addData("positonleftMotor", pulleyMotorL.getCurrentPosition());
            telemetry.addData("targetPosition", targetPosition);
            telemetry.addData("power", power);
            telemetry.update();
            pulleyMotorL.setPower(power);
            pulleyMotorR.setPower(power);
        }

        Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        Roll = robotOrientation.getRoll(AngleUnit.DEGREES);
        antiTip(Pitch);


    }



    public void stop(){
        OpmodeAvil = false;

    }
    //mecanum methods
    public void drive() {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        if(gamepad1.right_bumper){
            frontLeftPower = (   x + 2 * rx) / denominator;
            backLeftPower = (  - x + 2 * rx) / denominator;
            frontRightPower = ( - x - 2 * rx) / denominator;
            backRightPower = (  x - 2 * rx) / denominator;
        }
        else { // Calculate the mecanum motor powers
            frontLeftPower = (y + x + 2 * rx) / denominator;
            backLeftPower = (y - x + 2 * rx) / denominator;
            frontRightPower = (y - x - 2 * rx) / denominator;
            backRightPower = (y + x - 2 * rx) / denominator;
        }



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


        if (gamepad1.left_trigger>0.1) {
            motorFrontLeft.setPower(frontLeftPower * 0.4);
            motorBackLeft.setPower(backLeftPower * 0.4);
            motorFrontRight.setPower(frontRightPower * 0.4);
            motorBackRight.setPower(backRightPower * 0.4);
        }


        else if (gamepad1.right_trigger>0.1) {
            motorFrontLeft.setPower(frontLeftPower * 0.2);
            motorBackLeft.setPower(backLeftPower * 0.2);
            motorFrontRight.setPower(frontRightPower * 0.2);
            motorBackRight.setPower(backRightPower * 0.2);
        }
        else {
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }
        if (motorBackRight.getPower() == 0 && motorFrontRight.getPower() == 0 && motorBackLeft.getPower() == 0 && motorFrontLeft.getPower() == 0 && !isActivated) {
            motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void antiTip(double pitch) {
        if(Math.abs(pitch)>20){
            double pitchCorrection = 0.125*pitch;  // tune 0.125 p term later
            motorBackLeft.setPower(pitchCorrection);
            motorBackRight.setPower(pitchCorrection);
            motorFrontLeft.setPower(pitchCorrection);
            motorFrontRight.setPower(pitchCorrection);
        }

    }

}
