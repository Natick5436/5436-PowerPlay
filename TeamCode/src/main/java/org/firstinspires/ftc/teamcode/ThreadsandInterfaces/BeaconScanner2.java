package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class BeaconScanner2 extends OpenCvPipeline {

    public enum BarcodePosition {
        PINK,
        YELLOW,
        GREEN

    }

    boolean on;
    private OpenCvCamera cam;
    private Mat mat = new Mat();
    private Rect upperROI = new Rect(new Point(240, 120), new Point(304, 145));
    private Rect lowerROI = new Rect(new Point(240, 145), new Point(304, 170));

    private Mat upperMat;
    private Mat lowerMat;


    private Telemetry telemetry;
    private volatile BeaconPosition beaconPosition = BeaconPosition.PINK;

//    void inputToHSV(Mat input) {
//        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
//        Core.inRange(HSV, LOW, HIGH, filter);
//    }

    @Override
    public void init(Mat firstFrame) {

//        inputToHSV(firstFrame);

    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowerBound = new Scalar(25, 100, 100);
        Scalar upperBound = new Scalar(175, 255, 255);
        Core.inRange(mat, lowerBound, upperBound, mat);

        upperMat = mat.submat(upperROI);
        lowerMat = mat.submat(lowerROI);

        double upperValue = Math.round(Core.mean(upperMat).val[2] / 255);
        double lowerValue = Math.round(Core.mean(lowerMat).val[2] / 255);

        upperMat.release();
        lowerMat.release();
        mat.release();

        final double threshold = 10;
        if(upperValue > threshold){
            beaconPosition = BeaconPosition.PINK;
        }else if(lowerValue > threshold){
            beaconPosition = BeaconPosition.YELLOW;
        }else{
            beaconPosition = BeaconPosition.GREEN;
        }


        return mat;
    }

    public BeaconPosition getBeaconPosition() {
        return beaconPosition;
    }
}
