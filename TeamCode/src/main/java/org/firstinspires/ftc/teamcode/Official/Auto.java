package org.firstinspires.ftc.teamcode.Official;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.Mark11;
import org.firstinspires.ftc.teamcode.Robots.Mark12;
import org.firstinspires.ftc.teamcode.Robots.Mark13;
import org.firstinspires.ftc.teamcode.Robots.ScrimRobot;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BarcodeScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

//This auto is funni and completely stupid and got us 60 points
@Autonomous(name="TesterAuto",group="Autonomous")
public class Auto extends LinearOpMode {
    Mark13 robot;

    //OpenCvInternalCamera phoneCam;
    //BarcodeScanner pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Mark13(this, /*Mark_9.getSavedX()*/1.2, /*Mark_9.getSavedY()*/1.2, 0);

        robot.disableBrakes();
        /*String visionPlaceholder = "low";
        telemetry.addData("Status", "Ready");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new BarcodeScanner();
        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
            @Override
            public void onError(int errorCode)
            {
            }
        });*/

        while (!isStarted()) {
            telemetry.addData("x", robot.getX());
            telemetry.addData("y", robot.getY());
            telemetry.addData("angle", robot.getAngle());
            telemetry.update();
            if (isStopRequested()) {
                return;
            }
        }

        waitForStart();

        //robot.maneuverToPosition(2, 2, 0.4, 0);
        robot.maneuverToPosition(1.2,2,0.4,0);
        telemetry.addData("x", robot.getX());
        telemetry.addData("y", robot.getY());
        telemetry.update();
        sleep(8000);
        telemetry.addData("x", robot.getX());
        telemetry.addData("y", robot.getY());
        telemetry.update();



        robot.stopDrive();

    }
}