
package TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name = "LinearSlides")
@Config
public class LinearSlides extends OpMode {

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
    public static double smallHeight = 1500;
    public static double midHeight =2100;
    public static double tallHeight =4500;
    public static double motorPower =0.5;
    public static double targetPosition = 5;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void init() {
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





    }

    @Override
    public void loop()
    {

        if(gamepad2.dpad_up && pulleyMotorL.getCurrentPosition()<4100)
        {
            targetPosition = tallHeight;

        }
        if(gamepad2.dpad_down && pulleyMotorL.getCurrentPosition()>247)
        {
            targetPosition = smallHeight;

        }
        if(gamepad2.dpad_right)
        {
            targetPosition = midHeight;

        }

        if(gamepad2.right_bumper && pulleyMotorL.getCurrentPosition()<4400 )
        {

            pulleyMotorL.setPower(motorPower);
            pulleyMotorR.setPower(motorPower);
            targetPosition = pulleyMotorL.getCurrentPosition();
        }
        if(gamepad2.left_bumper && pulleyMotorL.getCurrentPosition() > 50)
        {
            pulleyMotorL.setPower(-motorPower);
            pulleyMotorR.setPower(-motorPower);
            targetPosition = pulleyMotorL.getCurrentPosition();

        }
        if(!gamepad2.right_bumper && !gamepad2.left_bumper && (Math.abs(targetPosition - pulleyMotorL.getCurrentPosition())<15))
        {

            pulleyMotorL.setPower(0);
            pulleyMotorR.setPower(0);

        }
        if(gamepad2.dpad_left)
        {
            targetPosition = 0;

        }

        double power = returnPower(targetPosition, pulleyMotorL.getCurrentPosition());
        telemetry.addData("positonrightMotor", pulleyMotorR.getCurrentPosition());
        telemetry.addData("positonleftMotor", pulleyMotorL.getCurrentPosition());
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("power", power);


        pulleyMotorL.setPower(power);
        pulleyMotorR.setPower(power);




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

}
