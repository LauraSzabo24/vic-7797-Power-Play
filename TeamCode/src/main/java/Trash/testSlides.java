package Trash;

public class testSlides {
}

/*package Trash;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@Config

@TeleOp(name = "Slides")
public class Slides extends OpMode
{
    //private Motor liftA;
    //private Motor liftB;
    private DcMotorEx liftA, liftB;
    int liftError = 0;
    int liftTargetPos = 0;

    private LiftPID liftPID1 = new LiftPID(.0075, 0, .0);

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void init(){
        //liftA = new Motor(hardwareMap, "LeftSlideMotor", Motor.GoBILDA.RPM_435);
        //liftB = new Motor(hardwareMap, "RightSlideMotor", Motor.GoBILDA.RPM_435);
        //liftA.setInverted(true);

        liftA = (DcMotorEx) hardwareMap.dcMotor.get("LeftSlideMotor");
        liftB = (DcMotorEx) hardwareMap.dcMotor.get("RightSlideMotor");
        liftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftB.setDirection(DcMotor.Direction.REVERSE);
        //liftA.setRunMode(Motor.RunMode.RawPower);
        //liftB.setRunMode(Motor.RunMode.RawPower);

//        liftA.setVeloCoefficients(0.5, 0.5,0.5);
//        liftB.setVeloCoefficients(0.5, 0.5,0.5);
    }

    @Override
    public void loop(){


        double leftAmount = gamepad2.left_trigger;
        double rightAmount = gamepad2.right_trigger;
        liftTargetPos = 300;

        liftError = liftTargetPos - liftA.getCurrentPosition();
        liftA.setPower(Range.clip(liftPID1.getCorrection(liftError), -.7, 1));
        liftB.setPower(Range.clip(liftPID1.getCorrection(liftError), -.7, 1));
//liftA.set(10);
//liftB.set(10);

        //telemetry.addData("P - ", liftPID1.getP());
        //telemetry.addData("I - ", liftPID1.getI());
        //telemetry.addData("D - ", liftPID1.getD());
        telemetry.addData("lift A Current Position", liftA.getCurrentPosition());
        telemetry.addData("lift B Current Position", liftB.getCurrentPosition());
        telemetry.addData("lift A Power", liftA.getPower());
        telemetry.addData("lift B Power", liftB.getPower());
        telemetry.addData("liftTargetPos", liftTargetPos);

        //liftA.set(leftAmount*1/2); //change 1/2 to good number
        //liftB.set(rightAmount*1/2);

        /*{
            pressure left = ....
            right = ...

            setpower(right*88)
            liftB.set(0.4);
            liftA.set(0.4);
        }
    }

            }*/