package org.firstinspires.ftc.teamcode.Official;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.Mark13;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BeaconPosition;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BeaconScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="WebcamTest",group="Autonomous")
public class WebcamTest extends LinearOpMode {
    private OpenCvInternalCamera cam;

    @Override
    public void runOpMode() throws InterruptedException {
        BeaconScanner scanner = new BeaconScanner(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Analysis", scanner.getBeaconPosition());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

    }


}
