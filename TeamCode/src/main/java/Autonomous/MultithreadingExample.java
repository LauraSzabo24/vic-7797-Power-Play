package Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import pipelines.AprilTagDetectionPipeline;

public MultithreadingExample extends LinearOpMode implements Runnable {
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
  
  //add it somewhere on code
  // use MultithreadingExample something = new MultithreadingExample();
     something.run();
  
    
  
