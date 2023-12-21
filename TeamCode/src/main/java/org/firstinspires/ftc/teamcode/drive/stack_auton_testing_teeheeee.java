package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "CenterStage_Auton")

public class stack_auton_testing_teeheeee extends LinearOpMode {
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

    boolean broken = false;


    ElapsedTime lifttimer = new ElapsedTime();
    private PIDFController controller;
    double p = 0.005;
    double i = 0.1;
    double d = 0.0005;
    double f = 0.0004;

    private DigitalChannel beambreakintake;
    private DigitalChannel beambreakouttake;

    enum State {
        SPIKE,
        PLACE,
        STACK,
        FORWARD,
        INTAKE,
        WAIT,
        LEAVE,
        IDLE
    }

    int pixelcount = 2;

    State currentState = State.IDLE;

    @Override
    public void runOpMode() {
        m_propDetector_left.SetWhichSide(1, 0.36, 0.40); //1=left side, 2=right side
//        0.58; // split it to left and right zones
//        private double m_zoneRange_split_rightSide = 0.36
//

        m_propDetector_right.SetWhichSide(2, 0.36, 0.40);
        initAllDevices();
        telemetry.setMsTransmissionInterval(50);

        beambreakintake = hardwareMap.get(DigitalChannel.class, "beambreakintake");
        beambreakintake.setMode(DigitalChannel.Mode.INPUT);
        beambreakouttake = hardwareMap.get(DigitalChannel.class, "beambreakouttake");
        beambreakouttake.setMode(DigitalChannel.Mode.INPUT);
        
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        controller = new PIDFController(p, i, d, f);
        controller.setPIDF(p,i,d, f);
        Pose2d startPose = new Pose2d(-63.75, -40.25, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        // Let's define our trajectories
        Trajectory left_spike = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(-90)))
                .build();
        TrajectorySequence left_spike_turn = drive.trajectorySequenceBuilder(left_spike.end())
                .forward(5)
                .turn(Math.toRadians(180))
                .build();
        Trajectory left_stack1 = drive.trajectoryBuilder(left_spike_turn.end(),true)
                .lineToLinearHeading(new Pose2d(-12, -60, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // Second trajectory
        // Ensure that we call left_stack1.end() as the start for this one
        Trajectory left_stack2 = drive.trajectoryBuilder(left_stack1.end())
                .lineToLinearHeading(new Pose2d(-12, -50, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory left_stack3 = drive.trajectoryBuilder(left_stack2.end())
                .lineToLinearHeading(new Pose2d(-12, -60, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory left_stack4 = drive.trajectoryBuilder(left_stack3.end())
                .lineToLinearHeading(new Pose2d(-12, -30, Math.toRadians(90)))
                .build();
        Pose2d poseEstimate =  drive.getPoseEstimate();


        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        // Define the angle for turn 2
        double turnAngle2 = Math.toRadians(720);


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
        waitTimer1.reset();
        currentState = State.SPIKE;
        drive.followTrajectory(left_spike);
        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case SPIKE:
                    if (!drive.isBusy()) {
                        currentState = State.PLACE;
                        drive.intake_pathway.setPower(-0.7);
                        waitTimer1.reset();

                    }
                    break;
                case PLACE:
                    if (waitTimer1.seconds() >= 1) {
                        currentState = State.STACK;
                        drive.intake_pathway.setPower(0);
                        drive.followTrajectoryAsync(left_stack1);
                    }
                    break;
                case STACK:
                    if (!drive.isBusy()) {
                        currentState = State.FORWARD;
                        drive.intake_pivotL.setPosition(0.375);
                        drive.intake_pivotR.setPosition(0.375);
                        drive.followTrajectoryAsync(left_stack2);
                    }
                    break;
                case FORWARD:
                    // Check if the drive class is busy followo04ing the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.INTAKE;
                        drive.followTrajectoryAsync(left_stack3);
                        drive.intake_pivotL.setPosition(0.37);
                        drive.intake_pivotR.setPosition(0.37);
                        drive.intake_pathway.setPower(1);
                        drive.intake_entrance.setPower(1);
                       
                    }
                    break;
                case INTAKE:
                    if (pixelcount>3){
                        waitTimer1.reset();
                        waitTime1 = 0.25;
                        drive.breakFollowing();
                        drive.setDrivePower(new Pose2d());
                        poseEstimate = drive.getPoseEstimate();
                        broken = true;
                        currentState = State.WAIT;

                    }else if (!drive.isBusy()) {
                        broken = false;
                        currentState = State.WAIT;

                        waitTimer1.reset();



                    }

                    break;
                case WAIT:
                    if(broken) {
                        left_stack4 = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(new Pose2d(-12, -30, Math.toRadians(90)))
                                .build();
                    }
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.LEAVE;
                        drive.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY);
                        drive.intake_pathway.setPower(-0.5);
                        drive.intake_entrance.setPower(-0.1);
                        drive.followTrajectoryAsync(left_stack4);
                    }
                    break;
                case LEAVE:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                        drive.intake_pivotL.setPosition(0.07);
                        drive.intake_pivotR.setPosition(0.07);
                        drive.intake_pathway.setPower(0);
                        drive.intake_entrance.setPower(0);
                        pixelcount = 0;
                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            //lift.update();

            if (!beambreakintake.getState() && pixelcount == 0) {
                pixelcount = 1;
            } else if (beambreakintake.getState() && pixelcount == 1) {
                pixelcount = 2;
            } else if (!beambreakintake.getState() && pixelcount == 2) {
                pixelcount = 3;
            } else if (beambreakintake.getState() && pixelcount == 3) {
                pixelcount = 4;
            }
        }

            // Read pose
            poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("beam", beambreakintake.getState());

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
                ( (sigNum_left == m_propDetector_left.SIG_0_NONE) && (sigNum_right == m_propDetector_left.SIG_1_LEFT))
        )
        {
            m_signalTag = m_propDetector_left.SIG_2_MIDDLE; //middle
        }
        else if (((sigNum_left == m_propDetector_left.SIG_0_NONE) && (sigNum_right == m_propDetector_left.SIG_3_RIGHT)) ||
                ((sigNum_left == m_propDetector_left.SIG_3_RIGHT) && (sigNum_right == m_propDetector_left.SIG_3_RIGHT)) ||
                ((sigNum_left == m_propDetector_left.SIG_1_LEFT) && (sigNum_right == m_propDetector_left.SIG_3_RIGHT)))
        {
            m_signalTag = m_propDetector_left.SIG_3_RIGHT;
        }
        else if (((sigNum_left == m_propDetector_left.SIG_1_LEFT) && (sigNum_right == m_propDetector_left.SIG_0_NONE)) ||
                ((sigNum_left == m_propDetector_left.SIG_0_NONE) && (sigNum_right == m_propDetector_left.SIG_0_NONE)))
            m_signalTag = m_propDetector_left.SIG_1_LEFT;
        else {
            m_signalTag = sigNum_left = sigNum_right = m_propDetector_left.SIG_0_NONE;
        }
        return;
    }

    private void tagToTelemetry(int detection) {
        telemetry.addLine(String.format("\nDetected tag ID = %d", detection));
    }

}