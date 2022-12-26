package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

public class BeaconScanner extends OpenCvPipeline {
    private OpenCvCamera cam;
    private Mat mat = new Mat();
    private Rect upperROI = new Rect(new Point(240, 120), new Point(304, 145));
    private Rect lowerROI = new Rect(new Point(240, 145), new Point(304, 170));

    private Mat upperMat;
    private Mat lowerMat;

    private BeaconPosition beaconPosition;
    private Telemetry telemetry;

    public BeaconScanner(HardwareMap hwMap, Telemetry t){
        telemetry = t;
        int camMonViewId = hwMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hwMap.appContext.getPackageName()
        );
        cam = OpenCvCameraFactory.getInstance().createWebcam(
                hwMap.get(WebcamName.class, "Webcam 1"),
                camMonViewId
        );
        cam.setPipeline(this);
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                cam.startStreaming(320,240);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        //cam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY);
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGBA2BGR);
        Scalar lowerBound = new Scalar(25, 186, 197);
        Scalar upperBound = new Scalar(175, 186, 197);

        upperMat = mat.submat(upperROI);
        lowerMat = mat.submat(lowerROI);

        double upperValue = Math.round(Core.mean(upperMat).val[2] / 255);
        double lowerValue = Math.round(Core.mean(lowerMat).val[2] / 255);

        mat.release();
        upperMat.release();
        lowerMat.release();

        final double threshold = 10;
        if(upperValue > threshold){
            beaconPosition = BeaconPosition.PINK;
        }else if(lowerValue > threshold){
            beaconPosition = BeaconPosition.YELLOW;
        }else{
            beaconPosition = BeaconPosition.GREEN;
        }

        return null;
    }

    public BeaconPosition getBeaconPosition() {
        return beaconPosition;
    }
}
