package TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;

import pipelines.AprilTagDetectionPipeline;


public class SlideFixerT extends LinearOpMode implements Runnable{
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
    public static Object lock_h =  new Object();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    //PID ends here

    //CLAW goes here
    private Servo rightServo;
    private Servo leftServo;
    //CLAW ends here

    public static boolean OPMODETRUE;
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



        //SERVO



    @Override
    public void runOpMode() throws InterruptedException {

    }
    @Override
    public void run() {
        while (opModeIsActive()) {
            while (Math.abs((getHeight() - pulleyMotorL.getCurrentPosition())) > 8) {
                double power = returnPower(getHeight(), pulleyMotorL.getCurrentPosition());
                pulleyMotorL.setPower(power);
                pulleyMotorR.setPower(power);
                telemetry.addData("position", pulleyMotorL.getCurrentPosition());
                telemetry.addData("power", power);
                telemetry.update();

            }

            while(gamepad2.right_bumper && pulleyMotorL.getCurrentPosition()<5000)
            {
                pulleyMotorL.setPower(1);
                pulleyMotorR.setPower(1);

            }


            while(gamepad2.left_bumper && pulleyMotorL.getCurrentPosition() > 0)
            {
                pulleyMotorL.setPower(-1);
                pulleyMotorR.setPower(-1);

            }
            targetPosition = pulleyMotorL.getCurrentPosition();

            pulleyMotorL.setPower(0);
            pulleyMotorR.setPower(0);
        }
    }


    public double returnPower(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error -lastError)/ timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;

    }

    public static void setHeight(double height) {
        synchronized (lock_h) {
            targetPosition = height;
        }
    }
    public static double getHeight() {
        synchronized (lock_h) {
            return targetPosition;
        }

    }
}


