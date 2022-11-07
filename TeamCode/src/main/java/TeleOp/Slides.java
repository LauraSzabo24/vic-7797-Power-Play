package TeleOp;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@TeleOp(name = "Slides")
public class Slides extends OpMode
{
    private Motor liftA;
    private Motor liftB;

    @Override
    public void init(){
        liftA = new Motor(hardwareMap, "LeftSlideMotor", Motor.GoBILDA.RPM_435);
        liftB = new Motor(hardwareMap, "RightSlideMotor", Motor.GoBILDA.RPM_435);
        liftA.setInverted(true);
        //liftA.setRunMode(Motor.RunMode.RawPower);
       // liftB.setRunMode(Motor.RunMode.RawPower);
        liftA.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        liftA.setVeloCoefficients(0.5, 0.5,0.5);
//        liftB.setVeloCoefficients(0.5, 0.5,0.5);
    }

    @Override
    public void loop(){
        double leftAmount = gamepad2.left_trigger;
        double rightAmount = gamepad2.right_trigger;

        liftA.set(leftAmount*1/2); //change 1/2 to good number
        liftB.set(rightAmount*1/2);

        liftA.set(PID.PIDMath(.3, liftA.getPositionCoefficient())); // figure out goal(the final distance/velocity we need)
        liftB.set(PID.PIDMath(.3, liftB.getPositionCoefficient())); // figure out what unit we are using for pulley motors
        /*{
            pressure left = ....
            right = ...

            setpower(right*88)
            liftB.set(0.4);
            liftA.set(0.4);
        }*/
    }
}