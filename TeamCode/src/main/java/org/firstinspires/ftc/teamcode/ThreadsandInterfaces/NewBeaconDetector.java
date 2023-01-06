package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class NewBeaconDetector {

    public enum BeaconColor {
        ORANGE,
        MAGENTA,
        GREEN
    }

    private Scalar lowerMagenta = new Scalar(80, 0, 130);
    private Scalar upperMagenta = new Scalar(255, 50, 255);
    private Scalar lowerGreen = new Scalar(50, 90, 40);
    private Scalar upperGreen = new Scalar(80, 240, 70);
    private Scalar lowerOrange = new Scalar(0, 0, 0);
    private Scalar upperOrange = new Scalar(255, 255, 255);

    public int magentaPix;
    public int orangePix;
    public int greenPix;

    public BeaconColor detect(Mat frame) {


            // Convert the frame to the HSV color space
            Mat hsv = new Mat();
            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);

            // Create masks for each color
            Mat maskYellow = new Mat();
            Core.inRange(hsv, lowerMagenta, upperMagenta, maskYellow);
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

        magentaPix = Core.countNonZero(maskYellow);
        greenPix = Core.countNonZero(maskGreen);
        orangePix = Core.countNonZero(maskOrange);

// Determine which color is the most intense
        if (magentaPix > greenPix && magentaPix > orangePix) {
            return BeaconColor.MAGENTA;
        } else if (greenPix > magentaPix && greenPix > orangePix) {
            return BeaconColor.GREEN;
        } else {
            return BeaconColor.ORANGE;
        }

    }

    public int getMagentaPix(){
        return magentaPix;
    }

    public int getOrangePix(){
        return orangePix;
    }

    public int getGreenPix(){
        return greenPix;
    }
}
