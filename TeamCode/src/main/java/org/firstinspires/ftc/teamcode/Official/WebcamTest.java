package org.firstinspires.ftc.teamcode.Official;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robots.Mark13;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BeaconPosition;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BeaconScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="WebcamTest",group="Autonomous")
public class WebcamTest extends LinearOpMode {
    Mark13 robot;

    @Override
    public void runOpMode() throws InterruptedException {
        BeaconScanner scanner = new BeaconScanner(hardwareMap, telemetry);
        String test;

//        while(!opModeIsActive()){
//            telemetry.addData("Camera", hardwareMap.get(WebcamName.class, "Webcam 1"));
//        }

        waitForStart();
        BeaconPosition beaconPosition = scanner.getBeaconPosition();
        scanner.stop();

        while (opModeIsActive())
        {
            telemetry.addData("Analysis",beaconPosition);
            if(beaconPosition==null){
                test="null";
            }else{
                test=beaconPosition.toString();
            }
            telemetry.addData("Test:", test);
            telemetry.addData("Camera", hardwareMap.get(WebcamName.class, "Webcam 1"));
            telemetry.addData("Hi:", scanner.getHi());
            telemetry.addData("On:", scanner.getOn());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

    }


}
