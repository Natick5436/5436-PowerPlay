package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BeaconPipeline extends OpenCvPipeline {
    private NewBeaconDetector beaconDetector = new NewBeaconDetector();

    public int yellowPix;
    public int orangePix;
    public int greenPix;

    NewBeaconDetector.BeaconColor beaconColor;

    @Override
    public Mat processFrame(Mat frame) {
        beaconColor = beaconDetector.detect(frame);
        yellowPix = beaconDetector.getYellowPix();
        orangePix = beaconDetector.getOrangePix();
        greenPix = beaconDetector.getGreenPix();

        return frame;
    }

    public NewBeaconDetector.BeaconColor getBeaconColor(){
        return beaconColor;
    }
    public int getYellowPix(){
        return yellowPix;
    }

    public int getOrangePix(){
        return orangePix;
    }

    public int getGreenPix(){
        return greenPix;
    }

}