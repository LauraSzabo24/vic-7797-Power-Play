package Trash;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class LiftPID {
    private double kp, ki, kd;
    private double totalError, lastError, lastTime;
    private ArrayList<Double> errors;

    public LiftPID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        lastTime = 0;
        errors = new ArrayList<Double>();
    }

    public double getP()
    {
        return kp;
    }

    public double getI()
    {
        return ki;
    }

    public double getD()
    {
        return kd;
    }

    public double getIError(double error) {
        if (Math.abs(error) < 1)
            totalError = 0;
        return ki * totalError;
    }
//dont use
    public boolean checkCorrection(double error){

        if(Math.abs(errors.get(errors.size()-1)) < Math.abs(error))
            return true;
        else
            return false;

    }

    //...
    public void setPD(double p, double d){
        kp = p;
        kd = d;
    }

    public double getCorrection(double error) {
        if(errors.size() != 0) {
            totalError += error;
            double potentialError = (kp * error) + getIError(error) + kd * (error - lastError);
            if (Math.abs(error) < 0.001 || (checkCorrection(potentialError))) {
                totalError -= error;
                return 0;
            }
            totalError -= error;
        }

        if (Math.abs(error) < 0.001) {
            return 0;
        }

        totalError += error;

        double output = (kp * error) + getIError(error) + kd * (error - lastError);

        lastError = error;
        errors.add(output);


        return output;
    }

    public ArrayList<Double> aman(){
        return errors;
    }
}