package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "CenterStage_Auton")

public class blue_auton_cam extends LinearOpMode {
    /* =============== camera variables  ===============*/
    private int m_signalTag = 0; // 1 - left, 2 - middle , 3 - right
    private OpenCvCamera m_camera_left = null;
    private OpenCvCamera m_camera_right = null;
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final String WEBCAM_LEFT = "Webcam 1";
    private static final String WEBCAM_RIGHT = "Webcam 2";
    private BluePropDetector m_propDetector_left = new BluePropDetector(CAMERA_WIDTH);
    private BluePropDetector m_propDetector_right = new BluePropDetector(CAMERA_WIDTH);

    ElapsedTime lifttimer = new ElapsedTime();
    private PIDFController controller;
    double p = 0.004;
    double i = 0.1;
    double d = 0.0005;
    double f = 0.0003;


    @Override
    public void runOpMode() {
        initAllDevices();
        telemetry.setMsTransmissionInterval(50);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        controller = new PIDFController(p, i, d, f);
        controller.setPIDF(p,i,d, f);
        Pose2d startPose = new Pose2d(-63.75, 16.75, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        Trajectory left_spike = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-40.00, 24, Math.toRadians(0.00)))
                .build();
        Trajectory left_depositbegin = drive.trajectoryBuilder(left_spike.end())
                .forward(5)
                .build();
        TrajectorySequence left_deposit_back = drive.trajectorySequenceBuilder(left_depositbegin.end())
                .back(10)
                .build();
        TrajectorySequence left_board = drive.trajectorySequenceBuilder(left_deposit_back.end())
                .lineToLinearHeading(new Pose2d(-46.00, 47.50, Math.toRadians(90.00)))
                .forward(5)
                .build();
        Trajectory leftback = drive.trajectoryBuilder(left_board.end())
                .back(5)
                .build();
        TrajectorySequence left_park = drive.trajectorySequenceBuilder(leftback.end()/*converge_end*/)
                .lineToLinearHeading(new Pose2d(-63, 44.00, Math.toRadians(0)))
                .strafeLeft(5)
                .build();

        Trajectory middle_spike = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35.00, 15, Math.toRadians(0)))
                .build();
        Trajectory middle_depositbegin = drive.trajectoryBuilder(middle_spike.end())
                .forward(5)
                .build();
        TrajectorySequence middle_deposit_back = drive.trajectorySequenceBuilder(middle_depositbegin.end())
                .back(10)
                .build();
        TrajectorySequence middle_board = drive.trajectorySequenceBuilder(middle_deposit_back.end())
                .lineToLinearHeading(new Pose2d(-37, 47.5, Math.toRadians(90.00)))
                .forward(5.5 )
                .build();
        Trajectory middleback = drive.trajectoryBuilder(middle_board.end())
                .back(5)
                .build();
        TrajectorySequence middle_park = drive.trajectorySequenceBuilder(middleback.end()/*converge_end*/)
                .lineToLinearHeading(new Pose2d(-63, 44.00, Math.toRadians(0)))
                .strafeLeft(5)
                .build();

        Trajectory right_spike = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-37.00, 12, Math.toRadians(270)))
                .build();
        Trajectory right_depositbegin = drive.trajectoryBuilder(right_spike.end())
                .forward(5)
                .build();
        TrajectorySequence right_deposit_back = drive.trajectorySequenceBuilder(right_depositbegin.end())
                .back(10)
                .build();
        TrajectorySequence right_board = drive.trajectorySequenceBuilder(right_deposit_back.end())
                .lineToLinearHeading(new Pose2d(-31, 47.5, Math.toRadians(90.00)))
                .forward(5)
                .build();
        Trajectory rightback = drive.trajectoryBuilder(right_board.end())
                .back(5)
                .build();
        TrajectorySequence right_park = drive.trajectorySequenceBuilder(rightback.end()/*converge_end*/)
                .lineToLinearHeading(new Pose2d(-63, 44.00, Math.toRadians(0)))
                .strafeLeft(5)
                .build();
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) { // detect signal on the sleeve
            detectSignal();
            boolean tagFound = false;

            if ((m_signalTag != 0) && (m_signalTag == m_propDetector_left.SIG_1_LEFT || m_signalTag == m_propDetector_left.SIG_2_MIDDLE || m_signalTag == m_propDetector_left.SIG_3_RIGHT)) {
                tagFound = true;
            }

            if (tagFound) {
                telemetry.addLine("Found Tag (1 = left, 2 = middle, 3 = right):");
                tagToTelemetry(m_signalTag);

            } else {
                telemetry.addLine("Don't see tag of interest!!!");
                sleep(200);
            }
            int sigZoneArea_lefttest, sigZoneArea_righttest;
            sigZoneArea_lefttest = m_propDetector_left.getMaxZoneArea();
            sigZoneArea_righttest = m_propDetector_right.getMaxZoneArea();

            telemetry.addData("cmpret", (double) Math.abs(sigZoneArea_lefttest - sigZoneArea_righttest) / Math.max(sigZoneArea_lefttest, sigZoneArea_righttest));
            telemetry.update();
            sleep(500);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (m_signalTag == 0) {
            // didn't detect the signal at init, now detect again
            detectSignal(); //?
            telemetry.addLine("Detecting Tag:\n");
        }
        telemetry.addLine("Tag detected (1 = left, 2 = middle, 3 = right):");
        tagToTelemetry(m_signalTag);
        telemetry.update();

///////////////////* Actually do something useful *////////////////////////////////////////////////////////////////////
        // firstly use back ramp to place a Purple pixel on the spike mark

        if (m_signalTag == 0 || m_signalTag == m_propDetector_left.SIG_1_LEFT) {
            /* left signal */
            drive.followTrajectory(left_spike);
            sleep(100);
            lifttimer.reset();
            drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            while((drive.lift_right.getCurrentPosition() > 360 || drive.lift_right.getCurrentPosition() < 330)&& lifttimer.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
                drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            drive.deposit.setPower(1);
            sleep(100);
            drive.followTrajectory(left_depositbegin);
            sleep(100);
            drive.deposit.setPower(0);
            sleep(100);
            drive.followTrajectorySequence(left_deposit_back);
            sleep(100);
            lifttimer.reset();
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            while (drive.lift_right.getCurrentPosition()  > 3 && lifttimer.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
                drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            }
            drive.lift_right.setPower(0);
            drive.lift_left.setPower(0);
            sleep(100);
            drive.followTrajectorySequence(left_board);
            sleep(100);
            lifttimer.reset();
            drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            while((drive.lift_right.getCurrentPosition() > 360 || drive.lift_right.getCurrentPosition() < 330)&& lifttimer.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
                drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            sleep(300);
            drive.deposit.setPower(1);
            sleep(300);
            lifttimer.reset();
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            while((drive.lift_right.getCurrentPosition() > 460 || drive.lift_right.getCurrentPosition() < 430)&& lifttimer.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
                drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            sleep(100);
            drive.followTrajectory(leftback);
            sleep(100);
            lifttimer.reset();
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.deposit.setPower(0);
            while (drive.lift_right.getCurrentPosition()  > 3 && lifttimer.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
                drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            }
            drive.lift_right.setPower(0);
            drive.lift_left.setPower(0);
            sleep(200);
            drive.followTrajectorySequence(left_park);
            telemetry.addLine("doRoute_left.");
        } else if (m_signalTag == m_propDetector_left.SIG_2_MIDDLE) {
            /* middle signal */
            drive.followTrajectory(middle_spike);
            sleep(100);
            lifttimer.reset();
            drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            while((drive.lift_right.getCurrentPosition() > 360 || drive.lift_right.getCurrentPosition() < 330)&& lifttimer.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
                drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            drive.deposit.setPower(1);
            sleep(100);
            drive.followTrajectory(middle_depositbegin);
            sleep(100);
            drive.deposit.setPower(0);
            sleep(100);
            drive.followTrajectorySequence(middle_deposit_back);
            sleep(100);
            lifttimer.reset();
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            while (drive.lift_right.getCurrentPosition()  > 3 && lifttimer.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
                drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            }
            drive.lift_right.setPower(0);
            drive.lift_left.setPower(0);
            sleep(100);
            drive.followTrajectorySequence(middle_board);
            sleep(100);
            lifttimer.reset();
            drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            while((drive.lift_right.getCurrentPosition() > 360 || drive.lift_right.getCurrentPosition() < 320)&& lifttimer.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
                drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            sleep(300);
            drive.deposit.setPower(1);
            sleep(300);
            lifttimer.reset();
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            while((drive.lift_right.getCurrentPosition() > 460 || drive.lift_right.getCurrentPosition() < 430)&& lifttimer.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
                drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            sleep(100);
            drive.followTrajectory(middleback);
            sleep(100);

            lifttimer.reset();
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.deposit.setPower(0);
            while (drive.lift_right.getCurrentPosition()  > 3 && lifttimer.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
                drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            }
            drive.lift_right.setPower(0);
            drive.lift_left.setPower(0);
            sleep(200);
            drive.followTrajectorySequence(middle_park);
            telemetry.addLine("doRoute_middle.");
        } else {
            /* right signal */
            drive.followTrajectory(right_spike);
            sleep(100);
            lifttimer.reset();
            drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            while((drive.lift_right.getCurrentPosition() > 360 || drive.lift_right.getCurrentPosition() < 330)&& lifttimer.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
                drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            drive.deposit.setPower(1);
            sleep(100);
            drive.followTrajectory(right_depositbegin);
            sleep(100);
            drive.deposit.setPower(0);
            sleep(100);
            drive.followTrajectorySequence(right_deposit_back);
            sleep(100);
            lifttimer.reset();
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            while (drive.lift_right.getCurrentPosition()  > 3 && lifttimer.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
                drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            }
            drive.lift_right.setPower(0);
            drive.lift_left.setPower(0);
            sleep(100);
            drive.followTrajectorySequence(right_board);
            sleep(100);
            lifttimer.reset();
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            while((drive.lift_right.getCurrentPosition() > 460 || drive.lift_right.getCurrentPosition() < 430)&& lifttimer.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
                drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
                telemetry.addData("right", drive.lift_right.getPower());
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            sleep(100);
            drive.followTrajectory(rightback);
            sleep(100);
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.deposit.setPower(0);

            while (drive.lift_right.getCurrentPosition()  > 3 && lifttimer.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
                drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            sleep(200);
            drive.followTrajectorySequence(right_park);
            telemetry.addLine("doRoute_right.");
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
        telemetry.addLine("Autonomous is completed.");
        telemetry.update();
    }

    private void initAllDevices() {


        /* =============== two cameras  ===============*/
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        m_camera_left = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_LEFT), viewportContainerIds[0]);
        m_camera_right = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_RIGHT), viewportContainerIds[1]);

        // Connect to the camera on the left side
        m_camera_left.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                m_camera_left.setPipeline(m_propDetector_left);
                m_camera_left.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        //        Connect to the camera on the right side
        m_camera_right.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                m_camera_right.setPipeline(m_propDetector_right);
                m_camera_right.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    private void doRoute_left() {

    }

    private void doRoute_middle() {


    }

    private void doRoute_right() {

    }

    private void detectSignal() {
        int sigNum_left, sigNum_right, sigZoneArea_left, sigZoneArea_right;
        m_signalTag = sigNum_left = sigNum_right = sigZoneArea_left = sigZoneArea_right = 0;

        double cmpRet = 0;
        sigNum_left = m_propDetector_left.getSignalNumber();
        sigNum_right = m_propDetector_right.getSignalNumber();
        sigZoneArea_left = m_propDetector_left.getMaxZoneArea();
        sigZoneArea_right = m_propDetector_right.getMaxZoneArea();
        if (Math.max(sigZoneArea_left, sigZoneArea_right) == 0) {
            m_signalTag = sigNum_left = sigNum_right = sigZoneArea_left = sigZoneArea_right = 0;
            return;
        }
        cmpRet = (double) Math.abs(sigZoneArea_left - sigZoneArea_right) / Math.max(sigZoneArea_left, sigZoneArea_right);

        if ((sigNum_left == 3 && sigNum_right == 1)) // && ( cmpRet <= 0.25))
            m_signalTag = 2; //middle
        else if ((sigZoneArea_left > sigZoneArea_right) && (cmpRet >= 0.3))
            m_signalTag = sigNum_left;
        else if ((sigZoneArea_left < sigZoneArea_right) && (cmpRet >= 0.3))
            m_signalTag = sigNum_right;
        else {
            m_signalTag = sigNum_left = sigNum_right = sigZoneArea_left = sigZoneArea_right = 0;
        }
        return;
    }

    private void tagToTelemetry(int detection) {
        telemetry.addLine(String.format("\nDetected tag ID = %d", detection));
    }

}