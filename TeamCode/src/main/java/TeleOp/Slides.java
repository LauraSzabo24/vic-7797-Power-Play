package TeleOp;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        liftA.setRunMode(Motor.RunMode.RawPower);
        liftB.setRunMode(Motor.RunMode.RawPower);
//        liftA.setVeloCoefficients(0.5, 0.5,0.5);
//        liftB.setVeloCoefficients(0.5, 0.5,0.5);
    }

    @Override
    public void loop(){
        double leftAmount = gamepad1.left_trigger;
        double rightAmount = gamepad1.right_trigger;

        /*if(gamepad1.a){
            liftB.set(0.5);
            liftA.set(0.5);
        }*/
    }
}