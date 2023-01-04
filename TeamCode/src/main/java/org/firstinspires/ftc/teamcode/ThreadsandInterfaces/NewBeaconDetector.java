package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

import java.util.ArrayList;
import java.util.List;

public class NewBeaconDetector {

    public enum BeaconColor {
        ORANGE,
        YELLOW,
        GREEN
    }

    private Scalar lowerYellow = new Scalar(20, 100, 100);
    private Scalar upperYellow = new Scalar(50, 255, 255);
    private Scalar lowerGreen = new Scalar(50, 90, 40);
    private Scalar upperGreen = new Scalar(80, 240, 70);
    private Scalar lowerOrange = new Scalar(25, 100, 110);
    private Scalar upperOrange = new Scalar(60, 130, 230);

    public int yellowPix;
    public int orangePix;
    public int greenPix;

    public BeaconColor detect(Mat frame) {


            // Convert the frame to the HSV color space
            Mat hsv = new Mat();
            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);

            // Create masks for each color
            Mat maskYellow = new Mat();
            Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);
            Mat maskGreen = new Mat();
            Core.inRange(hsv, lowerGreen, upperGreen, maskGreen);
            Mat maskOrange = new Mat();
            Core.inRange(hsv, lowerOrange, upperOrange, maskOrange);

            // Combine the masks
//            Mat mask = new Mat();
//            Core.add(maskYellow, maskGreen, mask);
//            Core.add(mask, maskOrange, mask);

            // Find contours in the mask
//            List<MatOfPoint> contours = new ArrayList<>();
//            Mat hierarchy = new Mat();
//            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            // If no contours are found, return NONE
//            if (contours.size() == 0) {
//                return BeaconColor.NONE;
//            }

            // Otherwise, determine which color the beacon is based on the contours
/*
            yellowPix = Core.countNonZero(maskYellow);
            orangePix = Core.countNonZero(maskOrange);
            greenPix = Core.countNonZero(maskGreen);

            if (Core.countNonZero(maskYellow) > Core.countNonZero(maskGreen) && Core.countNonZero(maskYellow) > Core.countNonZero(maskOrange)) {
                return BeaconColor.YELLOW;
            } else if (Core.countNonZero(maskGreen) > Core.countNonZero(maskYellow) && Core.countNonZero(maskGreen) > Core.countNonZero(maskOrange)) {
                return BeaconColor.GREEN;
            } else {
                return BeaconColor.ORANGE;
            }*/

        yellowPix = Core.countNonZero(maskYellow);
        greenPix = Core.countNonZero(maskGreen);
        orangePix = Core.countNonZero(maskOrange);

// Determine which color is the most intense
        if (yellowPix > greenPix && yellowPix > orangePix) {
            return BeaconColor.YELLOW;
        } else if (greenPix > yellowPix && greenPix > orangePix) {
            return BeaconColor.GREEN;
        } else {
            return BeaconColor.ORANGE;
        }

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
