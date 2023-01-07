package org.firstinspires.ftc.teamcode.Official;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.Mark13;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BeaconPipeline;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.NewBeaconDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="ThirdTest",group="Autonomous")
public class FourthTest extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    BeaconPipeline pipeline;
    NewBeaconDetector.BeaconColor beaconPipeline;
    Mark13 robot;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Mark13(this, /*Mark_9.getSavedX()*/3.3528, /*Mark_9.getSavedY()*/2.1336, 0);

        robot.disableBrakes();
        robot.disableBrakes();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new BeaconPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
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
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

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

        robot.simpleStrafe(0.1);
        sleep(985);
        robot.lF.setPower(0);
        robot.lB.setPower(0);
        robot.rF.setPower(0);
        robot.rB.setPower(0);
        robot.enableBrakes();

        robot.stopDrive();

    }
}