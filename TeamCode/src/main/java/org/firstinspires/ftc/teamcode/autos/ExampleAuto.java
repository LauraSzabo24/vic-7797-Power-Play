/*package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class ExampleAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double numberDetected = -1;

        while(!opModeIsActive()){
            // get the number of apriltag detected
            numberDetected = getNumber();
        }

        while(opModeIsActive() && !isStopRequested()){
            // first, move forward, strafe left, and deposit the preloaded cone

            if(numberDetected == 1){
                // park in zone 1
            }
            else if(numberDetected == 2){
                // park in zone 2
            }
            else {
                // park in zone 3
            }
        }
    }
}
*/