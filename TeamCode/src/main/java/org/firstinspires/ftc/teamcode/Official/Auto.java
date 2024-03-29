package org.firstinspires.ftc.teamcode.Official;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

import java.sql.Array;

//This auto is funni and completely stupid and got us 60 points
@Disabled
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
       //robot.strafe(1.2);
       robot.maneuverToPosition(1.2, 2, 0.55, 0);
        //robot.turn(0.3,90,10);
        //robot.armDown(0.2,200);
        //robot.maneuverToPosition(2,1.2,0.55,0);
        //robot.turn(0.5, Math.PI/2, Math.PI/60);
        //double[] array = {0.3, 0.3, 0};
        //robot.inverseKinematics(array);
        sleep(7000);
//        telemetry.addData("x", robot.getX());
//        telemetry.addData("y", robot.getY());
//        telemetry.update();
//        sleep(8000);
//        telemetry.addData("x", robot.getX());
//        telemetry.addData("y", robot.getY());
//        telemetry.update();



        robot.stopDrive();

    }
}