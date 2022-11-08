package TeleOp;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_RPM;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@TeleOp(name = "Slides")
public class Slides extends OpMode
{
    private Motor liftA;
    private Motor liftB;

    private final static double upPos = 10;
    private final static double downPos =-100;//fix up and down positions based on encoder readings

    private final double[] PIDF_COEFF = {0.0001, 0.001 , .001 , .001};

    private double targetPos;

    PIDFController liftPIDF = new PIDFController(PIDF_COEFF[0], PIDF_COEFF[1], PIDF_COEFF[2] , PIDF_COEFF[3]);

    @Override
    public void init(){
        liftA = new Motor(hardwareMap, "LeftSlideMotor", Motor.GoBILDA.RPM_435);
        liftB = new Motor(hardwareMap, "RightSlideMotor", Motor.GoBILDA.RPM_435);
        liftA.setInverted(true);

        liftA.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        liftB.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        liftA.setRunMode(Motor.RunMode.VelocityControl);
        liftB.setRunMode(Motor.RunMode.VelocityControl);

         //reset these coeff
          targetPos = upPos;
        //liftA.setRunMode(Motor.RunMode.RawPower);
       // liftB.setRunMode(Motor.RunMode.RawPower);

//        liftA.setVeloCoefficients(0.5, 0.5,0.5);
//        liftB.setVeloCoefficients(0.5, 0.5,0.5);
    }

    @Override
    public void loop(){


        if(gamepad1.a) {

        double correction = liftPIDF.calculate(liftB.getCurrentPosition(), targetPos);
        liftA.set(correction);
        liftB.set(correction);

        }


    }
}