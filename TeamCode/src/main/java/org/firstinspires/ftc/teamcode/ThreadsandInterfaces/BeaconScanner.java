package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BeaconScanner extends OpenCvPipeline {
    public enum BeaconPosition {
        ORANGE,
        YELLOW,
        GREEN
    }


    /*
     * Some color constants
     */

    Scalar yellowLowerBound = new Scalar(100, 100, 25);
    Scalar yellowUpperBound = new Scalar(255, 255, 175);

    Scalar greenLowerBound = new Scalar(50, 120, 95);
    Scalar greenUpperBound = new Scalar(95, 255, 160);

    Scalar orangeLowerBound = new Scalar(120, 50, 10);
    Scalar orangeUpperBound = new Scalar(200, 100, 50);


    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */


    /*
     * Working variables
     */
    private Mat mat = new Mat();
    private Rect scanRegion = new Rect(new Point(240, 120), new Point(304, 170));

    private Telemetry telemetry;
    private volatile BeaconPosition beaconPosition = BeaconPosition.ORANGE;


    Mat hsv = new Mat();
    Mat filter = new Mat();


    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */




    @Override
    public void init(Mat firstFrame) {
        Imgproc.cvtColor(firstFrame, hsv, Imgproc.COLOR_BGR2HSV);
    }

    @Override
    public Mat processFrame(Mat input) {

        // Create masks for each color
        Mat maskYellow = new Mat();
        Core.inRange(hsv, yellowLowerBound, yellowUpperBound, maskYellow);
        Mat maskGreen = new Mat();
        Core.inRange(hsv, greenLowerBound, greenUpperBound, maskGreen);
        Mat maskOrange = new Mat();
        Core.inRange(hsv, orangeLowerBound, orangeUpperBound, maskOrange);

        Mat mask = new Mat();
        Core.add(maskYellow, maskGreen, mask);
        Core.add(mask, maskOrange, mask);

//        Imgproc.rectangle(
//                input, // Buffer to draw on
//                new Point(240, 120), new Point(304, 170), // Second point which defines the rectangle
//                new Scalar(0, 255, 0), // The color the rectangle is drawn in
//                2); // Thickness of the rectangle lines
//
//        // Find contours in the mask
//        List<MatOfPoint> contours = new ArrayList<>();
//        Mat hierarchy = new Mat();
//        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//
//        for (MatOfPoint contour : contours) {
//            Rect rect = Imgproc.boundingRect(contour);
//            Imgproc.rectangle(input, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(0, 255, 0), 2);
//        }
//
//        // Otherwise, determine which color the beacon is based on the contours
//        if (Core.countNonZero(maskYellow) > Core.countNonZero(maskGreen) && Core.countNonZero(maskYellow) > Core.countNonZero(maskOrange)) {
//            beaconPosition = BeaconPosition.YELLOW;
//        } else if (Core.countNonZero(maskGreen) > Core.countNonZero(maskYellow) && Core.countNonZero(maskGreen) > Core.countNonZero(maskOrange)) {
//            beaconPosition = BeaconPosition.GREEN;
//        } else {
//            beaconPosition = BeaconPosition.ORANGE;
//        }

        return mask;
    }


    public BeaconPosition getAnalysis() {
        return beaconPosition;
    }
}
