package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
public class red_auton_cam_short extends LinearOpMode {
    /* =============== camera variables  ===============*/
    private int m_signalTag = 0; // 1 - left, 2 - middle , 3 - right
    private OpenCvCamera m_camera_left = null;
    private OpenCvCamera m_camera_right = null;
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final String WEBCAM_LEFT = "Webcam 1";
    private static final String WEBCAM_RIGHT = "Webcam 2";
    ElapsedTime lift = new ElapsedTime();
    private RedPropDetector m_propDetector_left = new RedPropDetector(CAMERA_WIDTH);
    private RedPropDetector m_propDetector_right = new RedPropDetector(CAMERA_WIDTH);
    private PIDFController controller;
    double p = 0.005;
    double i = 0.1;
    double d = 0.00055;
    double f = 0.0002;

    @Override
    public void runOpMode() {
        initAllDevices();
        telemetry.setMsTransmissionInterval(50);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        controller = new PIDFController(p, i, d, f);
        controller.setPIDF(p,i,d, f);
        Pose2d startPose = new Pose2d(63.75, 15.5, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        Trajectory right_spike = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(40.00, 24, Math.toRadians(180.00)))
                .build();
        TrajectorySequence right_deposit = drive.trajectorySequenceBuilder(right_spike.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(45, 51.00, Math.toRadians(90.00)))
                .build();
        TrajectorySequence right_park = drive.trajectorySequenceBuilder(right_deposit.end()/*converge_end*/)
                .lineToLinearHeading(new Pose2d(63, 44.00, Math.toRadians(180)))
                .strafeRight(8)
                .build();

        Trajectory middle_spike = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(38.00, 16, Math.toRadians(180)))
                .build();
        TrajectorySequence middle_deposit = drive.trajectorySequenceBuilder(middle_spike.end())
                .back(4)
                .lineToLinearHeading(new Pose2d(36, 51, Math.toRadians(90.00)))
                .build();
        TrajectorySequence middle_park = drive.trajectorySequenceBuilder(middle_deposit.end()/*converge_end*/)
                .lineToLinearHeading(new Pose2d(63, 44.00, Math.toRadians(180)))
                .strafeRight(8)
                .build();

        Trajectory left_spike = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(37.00, 12, Math.toRadians(270)))
                .build();
        Trajectory left_deposit = drive.trajectoryBuilder(left_spike.end())
                .lineToLinearHeading(new Pose2d(31, 51, Math.toRadians(90.00)))
                .build();
        TrajectorySequence left_park = drive.trajectorySequenceBuilder(left_deposit.end()/*converge_end*/)
                .lineToLinearHeading(new Pose2d(63, 44.00, Math.toRadians(180)))
                .strafeRight(8)
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
            drive.deposit.setPower(1);
            sleep(400);
            drive.deposit.setPower(0);
            drive.followTrajectory(left_deposit);
            sleep(100);
            lift.reset();
            drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            while((drive.lift_right.getCurrentPosition() > 360 || drive.lift_right.getCurrentPosition() < 330)&& lift.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
                drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            sleep(300);
            drive.deposit.setPower(1);
            sleep(300);
            lift.reset();
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            while((drive.lift_right.getCurrentPosition() > 460 || drive.lift_right.getCurrentPosition() < 430)&& lift.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
                drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            sleep(500);
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.deposit.setPower(0);
            while (drive.lift_right.getCurrentPosition()  > 3 && lift.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
                drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            sleep(200);
            drive.followTrajectorySequence(left_park);
            telemetry.addLine("doRoute_left.");
        } else if (m_signalTag == m_propDetector_left.SIG_2_MIDDLE) {
            /* middle signal */
            drive.followTrajectory(middle_spike);
            sleep(100);
            drive.deposit.setPower(1);
            sleep(400);
            drive.deposit.setPower(0);
            drive.followTrajectorySequence(middle_deposit);
            sleep(100);
            lift.reset();
            drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            while((drive.lift_right.getCurrentPosition() > 360 || drive.lift_right.getCurrentPosition() < 320)&& lift.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
                drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            sleep(300);
            drive.deposit.setPower(1);
            sleep(300);
            lift.reset();
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            while((drive.lift_right.getCurrentPosition() > 460 || drive.lift_right.getCurrentPosition() < 430)&& lift.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
                drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            sleep(500);
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.deposit.setPower(0);

            while (drive.lift_right.getCurrentPosition()  > 3 && lift.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
                drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            sleep(200);
            drive.followTrajectorySequence(middle_park);
            telemetry.addLine("doRoute_middle.");
        } else {
            /* right signal */
            drive.followTrajectory(right_spike);
            sleep(100);
            drive.deposit.setPower(1);
            sleep(400);
            drive.deposit.setPower(0);
            drive.followTrajectorySequence(right_deposit);
            sleep(100);
            lift.reset();
            drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            while((drive.lift_right.getCurrentPosition() > 360 || drive.lift_right.getCurrentPosition() < 320)&& lift.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
                drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),350 ), -0.5, 0.5));
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            sleep(300);
            drive.deposit.setPower(1);
            sleep(300);
            lift.reset();
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            while((drive.lift_right.getCurrentPosition() > 460 || drive.lift_right.getCurrentPosition() < 430)&& lift.seconds() < 3 && !isStopRequested()){
                drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
                drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
                telemetry.addData("right", drive.lift_right.getPower());
            }
            drive.lift_right.setPower(0.01);
            drive.lift_left.setPower(0.01);
            sleep(500);
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.deposit.setPower(0);

            while (drive.lift_right.getCurrentPosition()  > 3 && lift.seconds() < 3 && !isStopRequested()){
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

        if ((sigNum_left == 3 && sigNum_right == 1))
            m_signalTag = 2; //middle
        else if ((sigZoneArea_left > sigZoneArea_right) && (cmpRet >= 0.1))
            m_signalTag = sigNum_left;
        else if ((sigZoneArea_left < sigZoneArea_right) && (cmpRet >= 0.1))
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
