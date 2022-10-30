package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTesting")
public class TestTestTestServo extends OpMode
{

    private Servo rightServo;
    private Servo leftServo;

    @Override
    public void init(){

        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");

        //rightServo.setPosition(0);
        //leftServo.setPosition(0);
    }

    @Override
    public void loop(){

        if(gamepad1.b)
        {
            rightServo.setPosition(0.5);
            leftServo.setPosition(0.5); //0.41 0.34
        }
        if(gamepad1.a)
        {
            rightServo.setPosition(0.1);
            leftServo.setPosition(0.86);
        }
    }
}
