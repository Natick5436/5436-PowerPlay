/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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

/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */
@Autonomous(name="OfficialAuto", group = "Autonomous")
public class OfficialAuto extends LinearOpMode
{
    OpenCvInternalCamera phoneCam;
    BeaconPipeline pipeline;
    NewBeaconDetector.BeaconColor beaconPipeline;
    Mark13 robot;

    @Override
    public void runOpMode()
    {


        robot = new Mark13(this, /*Mark_9.getSavedX()*/1.2, /*Mark_9.getSavedY()*/1.2, 0);

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

        while (!isStarted())
        {
            beaconPipeline = pipeline.getBeaconColor();
            telemetry.addData("Analysis", beaconPipeline);
            telemetry.addData("True", true);
            telemetry.update();
            //phoneCam.stopStreaming();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        waitForStart();

        //robot.simpleStrafe(-0.2);
        sleep(800);
        robot.simpleStrafe(0);
        sleep(1000);

        robot.simpleForward(0.3);
        sleep(1250);
        robot.simpleForward(0);
        sleep(1500);

            if(beaconPipeline == NewBeaconDetector.BeaconColor.ORANGE){
                robot.simpleStrafe(-0.4);
                sleep(800);
                robot.simpleStrafe(0);
            }else if(beaconPipeline == NewBeaconDetector.BeaconColor.GREEN){
                sleep(100);
            }else/*Magenta*/{
                robot.simpleStrafe(0.4);
                sleep(1000);
                robot.simpleStrafe(0);
            }
            sleep(500);

//        robot.simpleForward(-0.3);
//        sleep(850);
//        robot.simpleForward(0);



        robot.stopDrive();

    }
}


