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

public class BluePropDetector extends OpenCvPipeline {
    private final boolean DEBUG_MODE = true;

    private int m_whichSide = 1; // 1=left, 2=right (default to be the left side)
    private final double zoneRange_topFilter = 0.45;
    private double m_zoneRange_split_leftSide = 0.58; // split it to left and right zones
    private double m_zoneRange_split_rightSide = 0.36; // split it to left and right zones

    private int m_signalNumber = 0;
    public final int SIG_0_NONE = 0;
    public final int SIG_1_LEFT = 1;
    public final int SIG_2_MIDDLE = 2;
    public final int SIG_3_RIGHT = 3;

    private final Scalar RED = new Scalar(255, 0, 0);
    private final Scalar BLUE = new Scalar(0, 0, 255);

    private int width = 0; // width of the image
    private int height = 0;

    private final Scalar lowHSV_2 = new Scalar(100,150,0);
    private final Scalar highHSV_2 = new Scalar(140,255,255);

    private Mat m_mat = null;

    private Telemetry telemetry;

    public BluePropDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * @param width The width of the image (check your camera)
     */
    public BluePropDetector(int width) {
        this.width = width;
    }
    public void SetWhichSide(int whichSide, double zoneRange_split_leftSide, double zoneRange_split_rightSide) {
//        m_zoneRange_split = zoneRange_split;
        m_whichSide = whichSide;
        m_zoneRange_split_leftSide = zoneRange_split_leftSide;
        m_zoneRange_split_rightSide = zoneRange_split_rightSide;
    }

    private int whichZone(Point center, int radius, int filterLine_top, int splitLine_LeftRight) {
        int zoneCode = 0;

        // firstly check if the circle is beyond the range
        // if ((center.y + radius) < filterLine_top)
        if (center.y < filterLine_top)
            zoneCode = SIG_0_NONE;
        else {
            if (center.x < splitLine_LeftRight)
                zoneCode = SIG_1_LEFT;
            else
                zoneCode = SIG_3_RIGHT;
        }
        return(zoneCode);
    }

    @Override
    public Mat processFrame(Mat input) {

        // get the size range for the image check
        if (width == 0) width = input.width();
        if (height == 0) height = input.height();

        // all the temporary shape related variables
        Mat thresh = null;

        // Make a working copy of the input matrix in HSV
        if (m_mat != null) {
            m_mat.release();
            m_mat = null; // make sure the memory for the previous matrix is released
        }

        m_mat = new Mat();
        Imgproc.cvtColor(input, m_mat, Imgproc.COLOR_RGB2HSV);

        // get two zones
        int splitLine_LeftRight = 0;
        if (m_whichSide == 1) // left side
            splitLine_LeftRight = (int)(input.width() * m_zoneRange_split_leftSide);
        else
            splitLine_LeftRight = (int)(input.width() * m_zoneRange_split_rightSide);

        int filterLine_top = (int)(input.height() * zoneRange_topFilter);

        thresh = new Mat();
        Core.inRange(m_mat, lowHSV_2, highHSV_2, thresh);

        // detect circle
        Mat gray = new Mat();
        Mat circles = new Mat();
        Point center = null;

        Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.medianBlur(gray, gray, 5);
        Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1.0,
                (double)gray.rows()/16, // minDist – Minimum distance between the centers of the detected circles.
                // change this value to detect circles with different distances to each other
                50, //param1 – First method-specific parameter. In case of #HOUGH_GRADIENT, it is the higher threshold of the two passed to the Canny edge detector (the lower one is twice smaller).
                30, //param2 – Second method-specific parameter.
                12, //minRadius – Minimum circle radius.
                40); //maxRadius – Maximum circle radius.
        // 100.0, 30.0, 10, 30); // change the last two parameters (min_radius & max_radius) to detect larger circles
        // 30,30,20,35); //zone 1
//                 50,30,14,35); //zone 2
        // 50,30,15,40); //zone 3
//                28.0,30.0,15,40); //11.26

        if (DEBUG_MODE && telemetry != null) {
            telemetry.addData("circles.cols()=", circles.cols());
            telemetry.addData("filterLine_top=", filterLine_top);
            telemetry.addData("splitLine_LeftRight=", splitLine_LeftRight);
            // telemetry.addData("minDist=", "%4.2f", (double)gray.rows()/16);
        }

        for (int x = 0; x < circles.cols(); x++) {
            double[] c = circles.get(0, x);
            center = new Point(Math.round(c[0]), Math.round(c[1]));
            int radius = (int) Math.round(c[2]);
            if (DEBUG_MODE && telemetry != null) {
                telemetry.addData("==>", "found a circle (%4.2f, %4.2f) radius=%d", center.x, center.y, radius) ;
            }
            m_signalNumber = whichZone(center, radius, filterLine_top, splitLine_LeftRight);
            if (SIG_0_NONE == m_signalNumber)
            { // beyond the range
                // circle it in Blue
                Imgproc.circle(m_mat, center, 1,BLUE, 3, 8, 0 );
                Imgproc.circle(m_mat, center, radius, BLUE, 2, 8, 0 );
            }
            else
            { // this is a good one
                // circle it in Red
                Imgproc.circle(m_mat, center, 1,RED, 3, 8, 0 );
                Imgproc.circle(m_mat, center, radius, RED, 2, 8, 0 );
                break;
            }
        }
        if (gray != null) {
            gray.release();
            gray = null;
        }
        if (circles != null) {
            circles.release();
            circles = null;
        }

        String str = "";

        if (m_signalNumber == SIG_1_LEFT)
            str = "Left";
        else if (m_signalNumber == SIG_3_RIGHT)
            str = "Right";
        else
            str = "None";

        // Adding text to the image
        Imgproc.putText(m_mat, str, new Point(120, 40), Imgproc.FONT_HERSHEY_SIMPLEX, 1, RED, 2);

        if (telemetry != null) {
            telemetry.addData("Signal number", m_signalNumber);
            telemetry.update();
            telemetry = null;
        }
        if (thresh != null) {
            thresh.release();
            thresh = null;
        }

        return m_mat;
    }

    public int getSignalNumber() {
        return this.m_signalNumber;
    }
    public String getSignalString() {
        if (this.m_signalNumber == SIG_1_LEFT) return "SIG_1_LEFT";
        else if (this.m_signalNumber == SIG_2_MIDDLE) return "SIG_2_MIDDLE";
        else if (this.m_signalNumber == SIG_3_RIGHT) return "SIG_3_RIGHT";
        else if (this.m_signalNumber == SIG_0_NONE) return "SIG_0_NONE";
        else return ("");
    }
}