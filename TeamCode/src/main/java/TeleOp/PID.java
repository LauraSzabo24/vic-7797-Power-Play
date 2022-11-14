package TeleOp;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "PID")

public class PID extends OpMode {

    DcMotorEx pulleyMotorR;
    DcMotorEx pulleyMotorL;

    ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;
    private double integralSum =0;

    public static double Kp =0.0125;
    public static double Ki =0.0; //.00005
    public static double Kd =0.0;
    public static double smallHeight = 0;
    public static double midHeight = 0;
    public static double tallHeight = 0;
    public static double motorSpeed = .5;


    public static double targetPosition = 1000;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init()  {


        TelemetryPacket packet = new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);
        pulleyMotorL = hardwareMap.get(DcMotorEx.class, "LeftSlideMotor");
        pulleyMotorR = hardwareMap.get(DcMotorEx.class, "RightSlideMotor");
        pulleyMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        pulleyMotorR.setDirection(DcMotorSimple.Direction.REVERSE);


        pulleyMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pulleyMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pulleyMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pulleyMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pulleyMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);







    }

    @Override
    public void loop() {

        if(gamepad2.dpad_down)
        {
            targetPosition = smallHeight;


        }
        if(gamepad2.dpad_right)
        {

            targetPosition = midHeight;

        }
        if(gamepad2.dpad_up)
        {

            targetPosition = tallHeight;

        }

        if (gamepad2.right_bumper && pulleyMotorL.getCurrentPosition() < 4100) {


            pulleyMotorL.setPower(motorSpeed);
            pulleyMotorR.setPower(motorSpeed);



            targetPosition = pulleyMotorL.getCurrentPosition();
        }
        else if(!gamepad2.right_bumper && !gamepad2.left_bumper && (Math.abs(targetPosition - pulleyMotorL.getCurrentPosition())<15)) {
            pulleyMotorL.setPower(0);
            pulleyMotorR.setPower(0);



        }

        if(gamepad2.left_bumper && pulleyMotorL.getCurrentPosition() > 240) {

            pulleyMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
            pulleyMotorR.setDirection(DcMotorSimple.Direction.FORWARD);

            pulleyMotorL.setPower(motorSpeed);
            pulleyMotorR.setPower(motorSpeed);

            pulleyMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
            pulleyMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

            targetPosition = pulleyMotorL.getCurrentPosition();

        }

        TelemetryPacket packet = new TelemetryPacket();
        double power = returnPower(targetPosition, pulleyMotorL.getCurrentPosition());
        packet.put("power", power);
        packet.put("position", pulleyMotorL.getCurrentPosition());
        packet.put("error", lastError);
        telemetry.addData("positon", pulleyMotorR.getCurrentPosition());
        telemetry.addData("positon", pulleyMotorL.getCurrentPosition());
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("power", power);


        pulleyMotorL.setPower(power);
        pulleyMotorR.setPower(power);

        dashboard.sendTelemetryPacket(packet);
    }

    public double returnPower(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error -lastError)/ timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output; //figure out how to connect dashboard

    }
}

