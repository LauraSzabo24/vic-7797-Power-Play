package TeleOp;



import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PID")

public class PID
{

    DcMotorEx pulley_motor_left;
    DcMotorEx pulley_motor_right;
    static double integralSum = 0;
    static double Kp = 1;
    static double Ki = 0;
    static double Kd = 0;

    static ElapsedTime timer = new ElapsedTime();
    static double lastError = 0;


    public static double PIDMath(double goal, double current)
    {
        double error = goal-current;
        integralSum += error * timer.seconds();
        double derivative = (error-lastError)/timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error *Kp) +(derivative *Kd) +(integralSum *Ki);
        return output;




    }
}





