package org.firstinspires.ftc.teamcode.Official;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.Mark11;
import org.firstinspires.ftc.teamcode.Robots.Mark12;
import org.firstinspires.ftc.teamcode.Robots.ScrimRobot;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BarcodeScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="UseThisAuto",group="Autonomous")
public class FirstAuto extends LinearOpMode {
    Mark12 robot;

    //OpenCvInternalCamera phoneCam;
    //BarcodeScanner pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Mark12(this, /*Mark_9.getSavedX()*/3.3528, /*Mark_9.getSavedY()*/2.1336, 0);

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
            //telemetry.addData("Analysis", pipeline.getAnalysis());
            //telemetry.addData("Position", pipeline.position);
            //telemetry.addData("x", robot.getX());

            telemetry.update();
            if (isStopRequested()) {
                return;
            }

        }

        waitForStart();

        robot.liftMotor.setPower(0.8);
        robot.angleMotor.setPower(0.8);

        robot.liftMotor.setTargetPosition(200);
        robot.angleMotor.setTargetPosition(75);
        sleep(1500);

        robot.SimpleForward(0.1);
        sleep(985);
        robot.SimpleForward(0.1);
        robot.enableBrakes();

        robot.stopDrive();

    }
}