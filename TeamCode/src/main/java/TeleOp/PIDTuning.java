package TeleOp;

//PID

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@TeleOp
public class PIDTuning extends OpMode {

    //PID junk
    DcMotorEx pulleyMotorR;
    DcMotorEx pulleyMotorL;

    public static double Kp = 0.0125;
    public static double Ki = 0.0; //.00005
    public static double Kd = 0.0;
    public static double Kf = 0.0;
    public static double targetPosition = 4000;


    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public void init() {
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


        //SERVO


    }




    @Override
    public void loop() {

            PIDFController pidSlide = new PIDFController(Kp,Ki,Kd,Kf);
            telemetry.addData("positionLL:", pulleyMotorL.getCurrentPosition());

            if(Math.abs(targetPosition - pulleyMotorL.getCurrentPosition()) > 12){

               double power = pidSlide.calculate(pulleyMotorL.getCurrentPosition(), targetPosition);
               pulleyMotorL.setPower(power);
               pulleyMotorR.setPower(power);
               telemetry.addData("Position:", pulleyMotorL.getCurrentPosition());
               telemetry.addData("error:", Math.abs(targetPosition - pulleyMotorL.getCurrentPosition()));

            }



        }



}
