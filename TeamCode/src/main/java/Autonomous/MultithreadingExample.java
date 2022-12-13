package Autonomous;

public class MultithreadingExample implements Runnable {
  @Override
  public void run() {
    try {
      double power = 0;
        while ((targetPosition - pulleyMotorL.getCurrentPosition()) > 12) {
          power = returnPower(targetPosition, pulleyMotorL.getCurrentPosition());
          pulleyMotorL.setPower(power);
          pulleyMotorR.setPower(power);
          targetPosition = pulleyMotorL.getCurrentPosition();
          telemetry.addData("positionLL:",pulleyMotorL.getCurrentPosition());
        }
    }
    catch (Exception e) {
        System.out.println("Something broke");
    }
  }
}
