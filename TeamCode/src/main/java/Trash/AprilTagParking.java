/*package Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AprilTagParking extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double numberDetected = -1;

        while(!opModeIsActive()){
            // get the number of apriltag detected

            numberDetected = AprilTagOp.tagNumber;
        }

        while(opModeIsActive() && !isStopRequested()){
            // first, move forward, strafe left, and deposit the preloaded cone

            if(numberDetected == 1){
                // park in zone 1
                telemetry.addData("PARK IN ZONE 1", numberDetected);
            }
            else if(numberDetected == 2){
                // park in zone 2
                telemetry.addData("PARK IN ZONE 2", numberDetected);
            }
            else {
                // park in zone 3
                telemetry.addData("PARK IN ZONE 3", numberDetected);
            }
        }
    }
}*/
//combine with AprilTagOp??
