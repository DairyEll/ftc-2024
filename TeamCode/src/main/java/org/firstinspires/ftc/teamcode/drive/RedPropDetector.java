package org.firstinspires.ftc.teamcode.drive;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RedPropDetector extends OpenCvPipeline {
    public final int SIG_1_LEFT = 1;
    public final int SIG_2_MIDDLE = 2;
    public final int SIG_3_RIGHT = 3;
    private final double zoneRange_split = 0.35; // this is the only difference from Blue detector
    private final double zoneRange_topFilter = 0.58;
    private final double zoneRange_bottomFilter = 0.13;
    private int m_signalNumber = 0;
    private int m_maxZoneArea = 0;
    private final boolean DEBUG_MODE = true;

    private final Scalar RED = new Scalar(255, 0, 0);
    private final Scalar BLUE = new Scalar(0, 0, 255);

    // https://cvexplained.wordpress.com/2020/04/28/color-detection-hsv/
    // lower boundary RED color range values; Hue (0 - 10)
    private final Scalar lowHSV_1 = new Scalar(0, 100, 20);
    private final Scalar highHSV_1 = new Scalar(10, 255, 255);

    // upper boundary RED color range values; Hue (160 - 180)
    private final Scalar lowHSV_2 = new Scalar(160,100,20);
    private final Scalar highHSV_2 = new Scalar(179,255,255);

    private Mat m_mat = null;
    private int width = 0; // width of the image
    private int height = 0;

    private Telemetry telemetry;

    public RedPropDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * @param width The width of the image (check your camera)
     */
    public RedPropDetector(int width) {
        this.width = width;
    }

    @Override
    public Mat processFrame(Mat input) {

        // get the size range for the image check
        if (width == 0) width = input.width();
        if (height == 0) height = input.height();

        // all the temporary shape related variables
        Mat edges = null;
        Mat lower_mask = null;
        Mat upper_mask = null;
        Mat full_mask = null;
        Mat thresh = null;

        Mat hierarchy = null;
        MatOfPoint2f[] contoursPoly = null;
        Rect[] boundRect = null;
        List<MatOfPoint> contours = null;

        // Make a working copy of the input matrix in HSV
        if (m_mat != null) {
            m_mat.release();
            m_mat = null; // make sure the memory for the previous matrix is released
        }

        m_mat = new Mat();
        Imgproc.cvtColor(input, m_mat, Imgproc.COLOR_RGB2HSV);

        // get three zones
        int line1 = (int)(input.width() * zoneRange_split);
        // int line2 = (int)(input.width() * (1 - zoneRange_right));
        int verticalFilterLine_top = (int)(input.height() * zoneRange_topFilter);
        int verticalFilterLine_bottom = (int)(input.height() * (1-zoneRange_bottomFilter));

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        lower_mask = new Mat();
        upper_mask = new Mat();
        full_mask = new Mat();
        thresh = new Mat();
        Core.inRange(m_mat, lowHSV_1, highHSV_1, lower_mask);
        Core.inRange(m_mat, lowHSV_2, highHSV_2, upper_mask);
        Core.addWeighted(lower_mask, 1, upper_mask, 1, 0, full_mask);

        // Bitwise-AND mask and original image
        Core.bitwise_and(input, input, thresh, full_mask);
        // Use Canny Edge Detection to find edges, you might have to tune the thresholds for hysteresis
        edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        contours = new ArrayList<>();
        hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        boundRect = new Rect[contours.size()];
        contoursPoly = new MatOfPoint2f[contours.size()];

        int maxWidth = 0;
        int maxHeight = 0;

        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            maxWidth = Math.max(maxWidth, boundRect[i].width);
            maxHeight = Math.max(maxHeight, boundRect[i].height);
        }

        if (DEBUG_MODE && telemetry != null) {
            telemetry.addData("input(width,height)=", "(%d, %d)", input.width(), input.height()); //input(width,height)= : (320, 180)
            telemetry.addData("1 lines", "(%d)", line1);
            telemetry.addData("verticalFilterLine_top", "(%d)", verticalFilterLine_top);
            telemetry.addData("verticalFilterLine_bottom", "(%d)", verticalFilterLine_bottom);

            telemetry.addData("maxWidth, maxHeight", "(%d, %d)", maxWidth, maxHeight);
            telemetry.addData("===", "===") ;
        }

        int leftZoneArea = 0;
        // int middleZoneCnt = 0;
        int rightZoneArea = 0;
        int tmpArea = 0;

        // ok, we've got the contours. let's check the shape
        int counter = 0;
        for (int i = 0; i != boundRect.length; i++)
        {
            if ((boundRect[i].x + boundRect[i].width) < line1) {

                tmpArea = getAreaAfterFilter(boundRect[i], verticalFilterLine_top,verticalFilterLine_bottom);
                leftZoneArea += tmpArea;
            }
            else if (boundRect[i].x >= line1){
                tmpArea = getAreaAfterFilter(boundRect[i], verticalFilterLine_top,verticalFilterLine_bottom);
                rightZoneArea += tmpArea;
            }
            else //if (boundRect[i].x >= line_mid_1 && boundRect[i].x <= line_mid_2)
            {
                // middleZoneCnt += boundRect[i].width * boundRect[i].height;
                tmpArea = getAreaAfterFilter(boundRect[i], verticalFilterLine_top, verticalFilterLine_bottom);
                leftZoneArea += tmpArea;
            }
//
//            if (boundRect[i].x < line1) {
//                tmpArea = getAreaAfterFilter(boundRect[i], verticalFilterLine_top);
//                leftZoneArea += tmpArea;
//            }else{
//                tmpArea = getAreaAfterFilter(boundRect[i], verticalFilterLine_top);
//                rightZoneArea += tmpArea;
//            }

            if (DEBUG_MODE) {
                if (tmpArea == 0)
                    Imgproc.rectangle(m_mat, boundRect[i], BLUE, 1);
                else
                    Imgproc.rectangle(m_mat, boundRect[i], RED, 1);
            }

            if (DEBUG_MODE && telemetry != null)
            {
                telemetry.addData("boundRect(x,y)", "(%d, %d)", boundRect[i].x, boundRect[i].y);
                telemetry.addData("boundRect(width, height)", "(%d, %d)", boundRect[i].width, boundRect[i].height);
                telemetry.addData("(leftZoneArea,rightZoneArea) =", "(%d, %d)", leftZoneArea,rightZoneArea);
                telemetry.addData("====", "====");
            }

        }
        if (DEBUG_MODE && telemetry != null) {
            telemetry.addData("(leftZoneArea,rightZoneArea) =", "(%d, %d)", leftZoneArea,rightZoneArea);
        }

        m_maxZoneArea = Math.max(leftZoneArea,rightZoneArea);

        String str = "";

        if (leftZoneArea == m_maxZoneArea)
        {
            m_signalNumber = SIG_1_LEFT;
            str = "Left";
        }
        else
        { //if can't see the left, it must be right
            m_signalNumber = SIG_3_RIGHT;
            str = "Right";
        }

        //Adding text to the image
        int scale = 1;
        int thickness = 2;
        Imgproc.putText(m_mat, str, new Point(120, 40), Imgproc.FONT_HERSHEY_SIMPLEX, scale, RED, thickness);

        if (telemetry != null) {
            telemetry.addData("Signal number", m_signalNumber);
            telemetry.update();
            telemetry = null;
        }
        if (edges != null) {
            edges.release();
            edges = null;
        }
        if (contours != null) {
            contours = null;
        }
        if (contoursPoly != null) contoursPoly = null;
        if (hierarchy != null) {
            hierarchy.release();
            hierarchy = null;
        }
        if (thresh != null) {
            thresh.release();
            thresh = null;
        }

        return m_mat;
    }
    private int getAreaAfterFilter(Rect rect, int verticalFilterLineTop, int verticalFilterLineBottom){
        int retRectArea = 0;
        if (rect.width == 1 || rect.height == 1)
            return retRectArea;

        if (rect.width > 10*rect.height)
            return retRectArea;

        // case 1
        if (rect.y + rect.height <= verticalFilterLineTop) {
            retRectArea = 0;
        }
        else if (rect.y >= verticalFilterLineBottom) {
            //case 2
            retRectArea = 0;
        }
        else if((rect.y >= verticalFilterLineTop) && (rect.y + rect.height) <= verticalFilterLineBottom) {
            //case 3
            retRectArea = rect.width * rect.height; // fully counted
        }
        else if ((rect.y <= verticalFilterLineTop) && ((rect.y + rect.height) <= verticalFilterLineBottom)) {
            // case 4
            retRectArea = rect.width * (rect.y + rect.height - verticalFilterLineTop) ;
        }
        else if ((rect.y >= verticalFilterLineTop) && (rect.y <= verticalFilterLineBottom) && ((rect.y + rect.height) > verticalFilterLineBottom)) {
            //case 5
            retRectArea = rect.width * (verticalFilterLineBottom - rect.y);
        }
        else if ((rect.y < verticalFilterLineTop) && ((rect.y + rect.height)  > verticalFilterLineBottom)) {
            //case 6
            retRectArea = rect.width * (verticalFilterLineBottom - verticalFilterLineTop);
        }
//      if (rect.y >= verticalFilterLineTop){
//        retRectArea = rect.width * rect.height;
//      }
//      else {
//        if (rect.y + rect.height < verticalFilterLineTop) {
//            retRectArea = 0;
//        }
//        else {
//           retRectArea = rect.width * (rect.y + rect.height - verticalFilterLineTop);
//        }
//      }
//
        return retRectArea;

    }
    public int getMaxZoneArea() {return this.m_maxZoneArea;}
    public int getSignalNumber() {
        return this.m_signalNumber;
    }
    public String getSignalString() {
        if (this.m_signalNumber == SIG_1_LEFT) return "SIG_1_LEFT";
        else if (this.m_signalNumber == SIG_2_MIDDLE) return "SIG_2_MIDDLE";
        else if (this.m_signalNumber == SIG_3_RIGHT) return "SIG_3_RIGHT";
        else return ("");
    }
}