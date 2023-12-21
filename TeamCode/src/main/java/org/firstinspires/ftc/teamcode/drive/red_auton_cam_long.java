package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "CenterStage_Auton")
public class red_auton_cam_long extends LinearOpMode {
    /* =============== camera variables  ===============*/
    private int m_signalTag = 0; // 1 - left, 2 - middle , 3 - right
    private OpenCvCamera m_camera_left = null;
    ElapsedTime lift = new ElapsedTime();
    private OpenCvCamera m_camera_right = null;
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final String WEBCAM_LEFT = "Webcam 1";
    private static final String WEBCAM_RIGHT = "Webcam 2";
    private RedPropDetector m_propDetector_left = new RedPropDetector(CAMERA_WIDTH);
    private RedPropDetector m_propDetector_right = new RedPropDetector(CAMERA_WIDTH);
    private PIDFController controller;
    double p = 0.005;
    double i = 0.1;
    double d = 0.00055;
    double f = 0.0002;

    @Override
    public void runOpMode() {
        m_propDetector_left.SetWhichSide(1, 0.58, 0.36); //1=left side, 2=right side
//        0.58; // split it to left and right zones
//        private double m_zoneRange_split_rightSide = 0.36
//

        m_propDetector_right.SetWhichSide(2, 0.58, 0.36);
        initAllDevices();
        telemetry.setMsTransmissionInterval(50);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        controller = new PIDFController(p, i, d, f);
        controller.setPIDF(p,i,d, f);
        Pose2d startPose = new Pose2d(63.75, -40.75, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        TrajectorySequence left_spike = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(40.00, -41), Math.toRadians(200.00))
                .build();
        TrajectorySequence left_deposit_back = drive.trajectorySequenceBuilder(left_spike.end())
                .back(20)
                .build();
        TrajectorySequence left_board = drive.trajectorySequenceBuilder(left_deposit_back.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(44.00, -36, Math.toRadians(180)))
                .forward(32)
                .turn(Math.toRadians(90))
                .splineTo(new Vector2d(15.62, 11.18), Math.toRadians(69.57))
                .splineTo(new Vector2d(31, 49.00), Math.toRadians(90.00))
                .forward(3)
                .build();
        Trajectory leftback = drive.trajectoryBuilder(left_board.end())
                .back(10)
                .build();
        TrajectorySequence left_park = drive.trajectorySequenceBuilder(leftback.end()/*converge_end*/)
                .turn(90)
                .build();

        Trajectory middle_spike = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(37.00, -38, Math.toRadians(160)))
                .build();
        TrajectorySequence middle_deposit_back = drive.trajectorySequenceBuilder(middle_spike.end())
                .back(10)
                .build();
        TrajectorySequence middle_board = drive.trajectorySequenceBuilder(middle_deposit_back.end())
                .lineToLinearHeading(new Pose2d(37, -60, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(12, -60, Math.toRadians(90.00)))
                .splineTo(new Vector2d(15.62, 11.18), Math.toRadians(69.57))
                .splineTo(new Vector2d(37, 49.00), Math.toRadians(90.00))
                .forward(3)
                .build();
        Trajectory middleback = drive.trajectoryBuilder(middle_board.end())
                .back(10)
                .build();
        TrajectorySequence middle_park = drive.trajectorySequenceBuilder(middleback.end()/*converge_end*/)
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence right_spike = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(5)
                .build();

        TrajectorySequence right_deposit_back = drive.trajectorySequenceBuilder(right_spike.end())
                .back(10)
                .build();
        TrajectorySequence right_board = drive.trajectorySequenceBuilder(right_deposit_back.end())
                .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(90.00)))
                .splineTo(new Vector2d(15.62, 11.18), Math.toRadians(69.57))
                .splineTo(new Vector2d(44.00, 49.00), Math.toRadians(90.00))
                .forward(3)
                .build();
        Trajectory rightback = drive.trajectoryBuilder(right_board.end())
                .back(10)
                .build();
        TrajectorySequence right_park = drive.trajectorySequenceBuilder(rightback.end()/*converge_end*/)
                .turn(Math.toRadians(90))
                .build();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) { // detect signal on the sleeve
            detectSignal();
            boolean tagFound = false;

            if (m_signalTag != m_propDetector_left.SIG_0_NONE) {
                tagFound = true;
            }
            if (tagFound) {
                telemetry.addLine("Found Tag (1 = left, 2 = middle, 3 = right):");
                tagToTelemetry(m_signalTag);

            } else {
                telemetry.addLine("Don't see tag of interest!!!");
                sleep(200);
            }

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
            drive.followTrajectorySequence(left_spike);
            sleep(100);
            drive.followTrajectorySequence(left_deposit_back);
            sleep(100);
            drive.followTrajectorySequence(left_board);
            sleep(100);
            lift.reset();
            drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            while((drive.lift_right.getCurrentPosition() > 370 || drive.lift_right.getCurrentPosition() < 330)&& lift.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
                drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            }
            drive.lift_right.setPower(0.02);
            drive.lift_left.setPower(0.02);
            sleep(100);
            drive.deposit.setPower(-1);
            sleep(200);
            drive.deposit.setPower(1);
            sleep(1000);
            drive.deposit.setPower(-1);
            sleep(200);
            drive.deposit.setPower(1);
            sleep(1000);
            drive.followTrajectory(leftback);
            sleep(100);
            lift.reset();
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.deposit.setPower(0);
            while (drive.lift_right.getCurrentPosition()  > 3 && lift.seconds() < 3 && !isStopRequested()){

                drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),0 ), -0.25, 0.25));
                drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),0 ), -0.25, 0.25));
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
            drive.followTrajectorySequence(middle_deposit_back);
            sleep(100);
            drive.followTrajectorySequence(middle_board);
            sleep(100);
            lift.reset();
            drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            while((drive.lift_right.getCurrentPosition() > 370 || drive.lift_right.getCurrentPosition() < 330)&& lift.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
                drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            sleep(100);
            drive.deposit.setPower(-1);
            sleep(200);
            drive.deposit.setPower(1);
            sleep(1000);
            drive.deposit.setPower(-1);
            sleep(200);
            drive.deposit.setPower(1);
            sleep(1000);
            drive.followTrajectory(middleback);
            sleep(100);

            lift.reset();
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.deposit.setPower(0);
            while (drive.lift_right.getCurrentPosition()  > 1 && lift.seconds() < 3 && !isStopRequested()){

                drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),0 ), -0.25, 0.25));
                drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),0 ), -0.25, 0.25));
            }
            drive.lift_right.setPower(0);
            drive.lift_left.setPower(0);
            sleep(200);
            drive.followTrajectorySequence(middle_park);
            telemetry.addLine("doRoute_middle.");
        } else {
            /* right signal */
            drive.followTrajectorySequence(right_spike);
            sleep(100);
            drive.followTrajectorySequence(right_deposit_back);
            sleep(100);
            drive.followTrajectorySequence(right_board);
            sleep(100);
            lift.reset();
            drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            while((drive.lift_right.getCurrentPosition() > 370 || drive.lift_right.getCurrentPosition() < 330)&& lift.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
                drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            sleep(100);
            drive.deposit.setPower(-0.9);
            sleep(200);
            drive.deposit.setPower(0.9);
            sleep(1000);
            drive.deposit.setPower(-0.9);
            sleep(200);
            drive.deposit.setPower(0.9);
            sleep(1000);
            drive.followTrajectory(rightback);
            sleep(100);
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.deposit.setPower(0);
            lift.reset();
            while (drive.lift_right.getCurrentPosition()  > 0 && lift.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),0 ), -0.25, 0.25));
                drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),0 ), -0.25, 0.25));
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
        int sigNum_left, sigNum_right;
        m_signalTag = sigNum_left = sigNum_right = m_propDetector_left.SIG_0_NONE;

        sigNum_left = m_propDetector_left.getSignalNumber();
        sigNum_right = m_propDetector_right.getSignalNumber();

        if (((sigNum_left == m_propDetector_left.SIG_3_RIGHT) && (sigNum_right == m_propDetector_left.SIG_1_LEFT)) ||
                ((sigNum_left == m_propDetector_left.SIG_3_RIGHT) && (sigNum_right == m_propDetector_left.SIG_0_NONE)) ||
                ((sigNum_left == m_propDetector_left.SIG_0_NONE) && (sigNum_right == m_propDetector_left.SIG_1_LEFT))
        ) {
            m_signalTag = m_propDetector_left.SIG_2_MIDDLE; //middle
        }
        else if (((sigNum_left == m_propDetector_left.SIG_1_LEFT) && (sigNum_right == m_propDetector_left.SIG_0_NONE)) ||
                ((sigNum_left == m_propDetector_left.SIG_1_LEFT) && (sigNum_right == m_propDetector_left.SIG_1_LEFT)))
        {
            m_signalTag = m_propDetector_left.SIG_1_LEFT;
        }
        else if (((sigNum_left == m_propDetector_left.SIG_0_NONE) && (sigNum_right == m_propDetector_left.SIG_3_RIGHT)) ||
                ((sigNum_left == m_propDetector_left.SIG_0_NONE) && (sigNum_right == m_propDetector_left.SIG_0_NONE)))
        {
            m_signalTag = m_propDetector_left.SIG_3_RIGHT;
        }
        else {
            m_signalTag = sigNum_left = sigNum_right = m_propDetector_left.SIG_0_NONE;
        }
        return;
    }

    private void tagToTelemetry(int detection) {
        telemetry.addLine(String.format("\nDetected tag ID = %d", detection));
    }


}
