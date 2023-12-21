package org.firstinspires.ftc.teamcode.drive;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name="drivecode", group="Iterative OpMode")
public class drivecode_blue extends OpMode {
    final double DESIRED_DISTANCE = 12; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.1 ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.1 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.05  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 1;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.75;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    private PIDFController controller;

    private DcMotor lift_left;
    private DcMotorEx lift_right;

    private DcMotorEx intake_entrance;
    private DcMotorEx intake_pathway;



    private Servo intake_pivotL;
    private Servo intake_pivotR;

    private CRServo deposit;

    boolean pose_reset = false;
    private Servo plane;

    public RevBlinkinLedDriver ledDriver;

    private DigitalChannel beambreakintake;
    private DigitalChannel beambreakouttake;
    private BNO055IMUNew imufordrive;

    private ElapsedTime deposittimer = new ElapsedTime();
    private ElapsedTime deposittimer1 = new ElapsedTime();
    private ElapsedTime deposittimer2 = new ElapsedTime();

    private ElapsedTime deposittimer3 = new ElapsedTime();
    boolean loaded = false;
    boolean loading = false;
    private boolean depositing = false;
    private boolean depositinghalf = false;
    private boolean depositingback = false;
    private boolean loadingback = false;

    private boolean loadingback2 = false;
    private boolean depositloading = false;

    boolean USE_WEBCAM_1 = true;
    int Portal_1_View_ID;
    boolean USE_WEBCAM_2 = true;
    int Portal_2_View_ID;
//    private AprilTagProcessor aprilTag1;
//    private VisionPortal visionPortal1;
    //AprilTagProcessor aprilTag2;
    //VisionPortal visionPortal2;
    private AprilTagDetection desiredTag1 = null;
    private AprilTagDetection desiredTag2 = null;

    boolean targetfound1     = false;
    boolean targetfound2     = false; // Set to true when an AprilTag target is detected
    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turn            = 0;

    int lift_level = 0;
    int board_pos;

    boolean position_change = false;
    boolean lift_pressed_up = false;

    boolean lift_pressed_up2 = false;
    boolean lift_pressed_down = false;
    //MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(10,10), 10));
    Vector<RevBlinkinLedDriver.BlinkinPattern> patterns = new Vector<RevBlinkinLedDriver.BlinkinPattern>(2,1);

    int assigned = 0;

    int pattern_stage;

    private ElapsedTime led_timer = new ElapsedTime();
    private ElapsedTime intake_timer = new ElapsedTime();
    private ElapsedTime intake_timer2 = new ElapsedTime();

    private ElapsedTime loading_timer = new ElapsedTime();
    private ElapsedTime loading_timer2 = new ElapsedTime();
    
    private boolean intake_button_test1 = false;
    private boolean intake_button_test2 = false;

    private boolean intake_button_test3 = false;
    private boolean intake_button_test4 = false;



    public static double p = 0.004;
    public static double i = 0.1;
    public static double d = 0.0005;
    public static double f = 0.0003;

    double intakeSpeed = 0;

    double power= 0;
    double yError = 0;
    double headingError = 0;
    double xError = 0;

    double pid;
    int error;

    int pixel_count = 0;
    int pixel_count2 = 0;

    List<Integer> lastTrackingEncPositions = new ArrayList<>();
    List<Integer> lastTrackingEncVels = new ArrayList<>();
    private StandardTrackingWheelLocalizer myLocalizer;
    Telemetry telemetrydash = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    boolean april_made = false;

    boolean exposure_set = false;

    double cam_multiplier = 1.1;
    double cam1_offset_x = 5.32185;
    double cam1_offset_y = -7.874016;
    double cam2_offset_x= -6.581693;
    double cam2_offset_y= -7.874016;
    boolean lift_slow = false;
    Pose2d botpose;

    ExposureControl exposureControl1;
    GainControl gainControl1;

    ExposureControl exposureControl2;
    GainControl gainControl2;
    double relocalizex = 0;
    double relocalizey = 0;
    double relocalizeheading = 0;










    @Override
    public void init() {
//        List myPortalsList;
//        myPortalsList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
//        Portal_1_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 0, false)).intValue();
//        Portal_2_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 1, false)).intValue();
//        if (!april_made) {
//            aprilTag1 = new AprilTagProcessor.Builder()
//                    //.setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
//                    .build();
//            visionPortal1 = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                    .setCameraResolution(new Size(640, 480))
//                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                    .addProcessor(aprilTag1)
//                    .setLiveViewContainerId(Portal_1_View_ID)
//                    .build();


//            aprilTag2 = new AprilTagProcessor.Builder()
//                    //.setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
//                    .build();
//            visionPortal2 = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
//                    .setCameraResolution(new Size(640, 480))
//                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                    .addProcessor(aprilTag2)
//                    .setLiveViewContainerId(Portal_2_View_ID)
//                    .build();
//            april_made = true;
//        }
        myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
        if(!pose_reset) {
            if (PoseStorage.currentPose == null) {
                myLocalizer.setPoseEstimate(new Pose2d(-63.75, 16.75, Math.toRadians(0)));
            } else {
                myLocalizer.setPoseEstimate(PoseStorage.currentPose);
            }
            pose_reset = true;
            telemetrydash.addData("pose storage", PoseStorage.currentPose);
            PoseStorage.currentPose = null;
        }

        //mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(10,10), 10));
        front_left = hardwareMap.get(DcMotorEx.class, "frontleft");
        back_left = hardwareMap.get(DcMotorEx.class, "backleft");
        front_right = hardwareMap.get(DcMotorEx.class, "frontright");
        back_right = hardwareMap.get(DcMotorEx.class, "backright");
        front_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        front_left.setDirection(DcMotorEx.Direction.REVERSE);
        back_left.setDirection(DcMotorEx.Direction.REVERSE);
        front_left.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);


        lift_right = hardwareMap.get(DcMotorEx.class, "rightlift");
        lift_left = hardwareMap.get(DcMotor.class, "leftlift");
        lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift_right.setDirection(DcMotorEx.Direction.REVERSE);
        lift_right.setPower(0);
        lift_left.setPower(0);
        controller = new PIDFController(p, i, d, f);
        controller.setPIDF(p,i,d, f);


        intake_entrance = hardwareMap.get(DcMotorEx.class, "intakeentrance");
        intake_entrance.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake_entrance.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake_entrance.setDirection(DcMotorEx.Direction.REVERSE);
        intake_entrance.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake_pathway = hardwareMap.get(DcMotorEx.class, "intakepathway");
        intake_pathway.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake_pathway.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake_pathway.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake_pivotL = hardwareMap.get(Servo.class, "intakepivotL");
        intake_pivotL.setDirection(Servo.Direction.REVERSE);
        intake_pivotR = hardwareMap.get(Servo.class, "intakepivotR");
        intake_pivotL.setPosition(0.07);
        intake_pivotR.setPosition(0.07);

        deposit = hardwareMap.get(CRServo.class, "deposit");
        deposit.setDirection(DcMotorEx.Direction.FORWARD);

        plane = hardwareMap.get(Servo.class, "plane");



        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "ledDriver");
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);

        imufordrive = hardwareMap.get(BNO055IMUNew.class, "imu");

        beambreakintake = hardwareMap.get(DigitalChannel.class, "beambreakintake");
        beambreakintake.setMode(DigitalChannel.Mode.INPUT);
        beambreakouttake = hardwareMap.get(DigitalChannel.class, "beambreakouttake");
        beambreakouttake.setMode(DigitalChannel.Mode.INPUT);



        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));




//            exposureControl.setMode(ExposureControl.Mode.ContinuousAuto);
//            exposureControl.setExposure(20, TimeUnit.MILLISECONDS);
//            gainControl.setGain(gainControl.getMaxGain());






//            exposureControl.setMode(ExposureControl.Mode.ContinuousAuto);
//            exposureControl.setExposure(20, TimeUnit.MILLISECONDS);
//            gainControl.setGain(gainControl.getMaxGain());
//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//            }
//            exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
//            gainControl.setGain(250);


//        telemetry.addData("cam1 ", visionPortal1.getCameraState());
        //telemetry.addData("cam2 ", visionPortal2.getCameraState());
        telemetry.update();
        telemetrydash.update();


        //Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imufordrive.initialize(parameters);
        imufordrive.resetYaw();


        plane.setPosition(0);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        myLocalizer.update();
        botpose = myLocalizer.getPoseEstimate();











    }

    /**
     *
     */
    @Override
    public void loop() {
        targetfound1 = false;
        targetfound2 = false;
//        if(visionPortal1.getCameraState() == VisionPortal.CameraState.STREAMING/* || visionPortal2.getCameraState() == VisionPortal.CameraState.STREAMING*/) {
//            if (!exposure_set) {
//                exposureControl1 = visionPortal1.getCameraControl(ExposureControl.class);
//                gainControl1 = visionPortal1.getCameraControl(GainControl.class);
////                exposureControl2 = visionPortal2.getCameraControl(ExposureControl.class);
////                gainControl2 = visionPortal2.getCameraControl(GainControl.class);
//                if (exposureControl1.getMode() != ExposureControl.Mode.Manual) {
//                    exposureControl1.setMode(ExposureControl.Mode.Manual);
//                }
//                exposureControl1.setExposure(10, TimeUnit.MILLISECONDS);
//                gainControl1.setGain(200);
//
////                if (exposureControl2.getMode() != ExposureControl.Mode.Manual) {
////                    exposureControl2.setMode(ExposureControl.Mode.Manual);
////                }
////                exposureControl2.setExposure(15, TimeUnit.MILLISECONDS);
////                gainControl2.setGain(200);
////                exposure_set = true;
//            }
//        }


//        if (lift_level > 0) {
//            if(visionPortal1.getCameraState() != VisionPortal.CameraState.STREAMING) {
//                visionPortal1.resumeStreaming();
////                visionPortal2.resumeStreaming();
//            }
//        } else {
//            if(visionPortal1.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_READY) {
//                visionPortal1.stopStreaming();
//                //visionPortal2.stopStreaming();
//            }
//        }
        //visionPortal2.stopStreaming();
//        if (visionPortal1.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_READY) {
//            visionPortal1.stopStreaming();
//            //visionPortal2.stopStreaming();
//        }


        //gamepad1 code


        //drivetrain code
//        List<AprilTagDetection> currentDetections1 = aprilTag1.getDetections();
//        if (lift_level == 0) {
//            targetfound1 = false;
//            targetfound2 = false;
//            currentDetections1 = new ArrayList<AprilTagDetection>();
//        }
//        for (AprilTagDetection detection1 : currentDetections1) {
//            if ((detection1.metadata != null) &&
//                    (detection1.id == 1)) {
//                targetfound1 = true;
//                desiredTag1 = detection1;
//
//                break;  // don't look any further.
//            } else if ((detection1.metadata != null) &&
//                    (detection1.id == 2)) {
//                targetfound1 = true;
//                desiredTag1 = detection1;
//                break;  // don't look any further.
//            } else if ((detection1.metadata != null) &&
//                    (detection1.id == 3)) {
//                targetfound1 = true;
//                desiredTag1 = detection1;
//                break;  // don't look any further.
//            } else {
//                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection1.id);
//
//
//            }
//        }
//        if (lift_level == 0) {
//            targetfound1 = false;
//            targetfound2 = false;
//        }
//
//        if (targetfound1) {
//            telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
//            telemetry.addData("Target", "ID %d (%s)", desiredTag1.id, desiredTag1.metadata.name);
//            telemetry.addData("x", "%5.1f inches", desiredTag1.ftcPose.x);
//            telemetry.addData("y", "%5.1f inches", desiredTag1.ftcPose.y * cam_multiplier);
//            telemetry.addData("Bearing", "%3.0f degrees", desiredTag1.ftcPose.bearing);
//            telemetry.addData("Yaw", "%3.0f degrees", desiredTag1.ftcPose.yaw);
//
//        }

//        List<AprilTagDetection> currentDetections2 = aprilTag2.getDetections();
//        if (lift_level == 0) {
//            targetfound1 = false;
//            targetfound2 = false;
//            currentDetections2 = new ArrayList<AprilTagDetection>();
//        }
//        for (AprilTagDetection detection2 : currentDetections2) {
//            if ((detection2.metadata != null) &&
//                    (detection2.id == 3)) {
//                targetfound2 = true;
//                desiredTag2 = detection2;
//
//                break;  // don't look any further.
//            } else if ((detection2.metadata != null) &&
//                    (detection2.id == 2)) {
//                targetfound2 = true;
//                desiredTag2 = detection2;
//                  // don't look any further.
//            } else if ((detection2.metadata != null) &&
//                    (detection2.id == 1)) {
//                targetfound2 = true;
//                desiredTag2 = detection2;
//                  // don't look any further.
//            } else {
//                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection2.id);
//
//
//            }
//        }
        if (lift_level == 0) {
            targetfound1 = false;
            targetfound2 = false;
        }

        if (targetfound2) {
            telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Target", "ID %d (%s)", desiredTag2.id, desiredTag2.metadata.name);
            telemetry.addData("x", "%5.1f inches", desiredTag2.ftcPose.x);
            telemetry.addData("y", "%5.1f inches", desiredTag2.ftcPose.y * cam_multiplier);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag2.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag2.ftcPose.yaw);

        }

        double botHeading = ((imufordrive.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))/* +autonomous1.anglestored+ autonomous2.anglestored*/);
        telemetry.addData("imu heading", botHeading);
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        if (gamepad1.back) {
            imufordrive.resetYaw();

        }
        if (botpose.getY() < 0) {
            board_pos = 0;
        }


        telemetry.addData("updated pose", new Pose2d(relocalizex, relocalizey, Math.toRadians(relocalizeheading)));
        if (targetfound1/* || targetfound2*/) {
            /*if (targetfound1 && targetfound2) {
                relocalizex = -cam_multiplier * ((desiredTag1.metadata.fieldPosition.get(1) - desiredTag1.ftcPose.x - cam1_offset_x) + (desiredTag2.metadata.fieldPosition.get(1) - desiredTag2.ftcPose.x - cam2_offset_x)) / 2;
                relocalizey = cam_multiplier * ((61.5 - desiredTag1.ftcPose.y + cam1_offset_y) + (61.5 - desiredTag2.ftcPose.y + cam2_offset_y)) / 2;
                relocalizeheading = ((90 - (desiredTag1.ftcPose.bearing + Math.atan(desiredTag1.ftcPose.x/desiredTag1.ftcPose.y))) + (90 - (desiredTag2.ftcPose.bearing + Math.atan(desiredTag2.ftcPose.x/desiredTag2.ftcPose.y)))) / 2;
                myLocalizer.setPoseEstimate(new Pose2d(relocalizex, relocalizey, Math.toRadians(relocalizeheading)));

            } else*/ if (targetfound1) {
                if(botpose.getY() < 45) {
                    double vectorx = (-cam1_offset_x - desiredTag1.ftcPose.range * Math.sin(Math.toRadians(desiredTag1.ftcPose.bearing)));
                    relocalizex = -desiredTag1.metadata.fieldPosition.get(1) - vectorx;
                    double vectory = (-cam1_offset_y + desiredTag1.ftcPose.range * Math.cos(Math.toRadians(desiredTag1.ftcPose.bearing)));
                    relocalizey = 61.5 - vectory;
                    relocalizeheading = 90 - desiredTag1.ftcPose.yaw;
                    myLocalizer.setPoseEstimate(new Pose2d(relocalizex, relocalizey, Math.toRadians(relocalizeheading)));
                    telemetry.addData("vectorx", vectorx);
                    telemetry.addData("vectory", vectory);
                    telemetry.addData("vectorheading", Math.toDegrees(-Math.atan(vectorx / vectory)));
                }


            } else {

//                relocalizex =-(desiredTag2.metadata.fieldPosition.get(1) - desiredTag2.ftcPose.x - cam2_offset_x);
//                relocalizey = (61.5 - desiredTag2.ftcPose.y  * cam_multiplier + cam2_offset_y);
//                relocalizeheading = (90 - (desiredTag2.ftcPose.bearing + Math.atan(desiredTag2.ftcPose.x/desiredTag2.ftcPose.y)));
//                myLocalizer.setPoseEstimate(new Pose2d(relocalizex, relocalizey, Math.toRadians(relocalizeheading)));
            }
        }
        if (lift_level % 2 == 0) {
            board_pos = Range.clip(board_pos, -3, 3);
            xError = -36 - botpose.getX() - 3.5 * board_pos;
        } else {
            board_pos = Range.clip(board_pos, -2, 3);
            xError = (-36 + 3.5 / 2) - botpose.getX() - 3.5 * board_pos;
        }
        yError = 52 - botpose.getY();
        if(Math.toDegrees(botpose.getHeading()) > 180) {
            headingError = 90 - Math.toDegrees(botpose.getHeading()) + 180;
        }else{
            headingError = 90 - Math.toDegrees(botpose.getHeading());
        }

        if (gamepad1.x && botpose.getY() > 12) {



            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.





            if (!position_change && gamepad1.left_stick_x > 0.3) {
                position_change = true;
                board_pos -= 1;
            } else if (!position_change && gamepad1.left_stick_x < -0.3) {
                position_change = true;
                board_pos += 1;
            } else if (position_change && (gamepad1.left_stick_x < 0.3 && gamepad1.left_stick_x > -0.3)) {
                position_change = false;
            }


            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(yError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-xError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            double leftFrontPower = drive - strafe - turn;
            double rightFrontPower = drive + strafe + turn;
            double leftBackPower = drive + strafe - turn;
            double rightBackPower = drive - strafe + turn;

            front_left.setPower(2500 * leftFrontPower);
            back_left.setPower(2500 * leftBackPower);
            front_right.setPower(2500 * rightFrontPower);
            back_right.setPower(2500 * rightBackPower);

            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        } else {
            if(gamepad1.left_bumper){
                front_left.setPower(frontLeftPower * 0.3);
                back_left.setPower(backLeftPower * 0.3);
                front_right.setPower(frontRightPower * 0.3);
                back_right.setPower(backRightPower * 0.3);
            }else {
                front_left.setPower(frontLeftPower);
                back_left.setPower(backLeftPower);
                front_right.setPower(frontRightPower);
                back_right.setPower(backRightPower);
            }
        }
        if(depositing){
            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }else if (depositloading || loadingback2) {
            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }else if(pixel_count == 1){
            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);
        }else if (pixel_count == 2){
            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }else {
            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        }

        //deposit code
        if (gamepad1.right_bumper && !depositing) {
            deposittimer.reset();
            depositing = true;
            depositloading = false;
            deposit.setPower(0.8);
            if (pixel_count2 > 0) {
                pixel_count2 -= 1;
                pixel_count -= 1;
            }

        } else if (beambreakouttake.getState() && depositing && !depositloading) {
            deposittimer.reset();
            depositing = false;

            if(pixel_count == 1) {
                deposit.setPower(-1);
                loadingback2 = true;
                loading_timer.reset();

            }else{
                deposit.setPower(0);
                depositloading = false;
                loadingback2 = false;
            }
        } else if (loading_timer.seconds() > 0.2 && loadingback2) {
            loadingback2 = false;
            depositloading = true;
            deposit.setPower(0.25);
            loading_timer2.reset();

        } else if(!beambreakouttake.getState() && depositloading && !depositing){
            deposit.setPower(0);
            depositloading = false;

        }else if (loading_timer2.seconds() > 1  && depositloading && !depositing){
            loading_timer.reset();
            depositloading = false;
            loadingback2 = true;
            deposit.setPower(-1);
        }else if (!depositing && !depositinghalf && !loading && !depositingback && !depositloading && !loadingback2) {
            deposit.setPower(0);
        }

        if (gamepad1.b && !depositinghalf) {
            deposittimer.reset();
            depositinghalf = true;
            deposit.setPower(0.8);
            if (pixel_count2 > 0) {
                pixel_count2 -= 1;
                pixel_count -= 1;
            }

        } else if (deposittimer.seconds() > 0.2 && depositinghalf) {
            deposittimer.reset();
            depositinghalf = false;
            deposit.setPower(0);
        } else if (!depositinghalf && !depositing && !loading && !depositingback&& !depositloading && !loadingback2) {
            deposit.setPower(0);
        }

        if (gamepad1.a && !depositingback) {
            deposittimer.reset();
            depositingback = true;
            deposit.setPower(-0.8);
            if (pixel_count2 > 0) {
                pixel_count2 -= 1;
                pixel_count -= 1;
            }

        } else if (deposittimer.seconds() > 0.2 && depositingback) {
            deposittimer.reset();
            depositingback = false;
            deposit.setPower(0);
        } else if (!depositinghalf && !depositing && !loading && !depositingback&& !depositloading&& !loadingback2) {
            deposit.setPower(0);
        }

        if (pixel_count > 0 && deposittimer1.seconds() > 1 && !loaded) {
            if (!loading) {
                deposittimer2.reset();
                loadingback = false;
                loading = true;
                deposit.setPower(0.2);

            } else if (!beambreakouttake.getState() && loading) {
                deposittimer2.reset();
                loadingback = false;
                loading = false;
                deposit.setPower(0);
                loaded = true;
            }else if (deposittimer2.seconds() > 2){
                if(!loadingback) {
                    loadingback = true;
                    deposittimer3.reset();
                    deposit.setPower(-1);
                }else if (deposittimer3.seconds() > 1){
                    deposit.setPower(0.2);
                    deposittimer2.reset();
                    loadingback =false;
                }
            }
        }
        if (pixel_count == 0) {
            deposittimer1.reset();
            loaded = false;
        }
        //intake code
        if (!beambreakintake.getState() && pixel_count == 0) {
            pixel_count = 1;
        } else if (beambreakintake.getState() && pixel_count == 1) {
            pixel_count2 = 1;
        } else if (!beambreakintake.getState() && pixel_count2 == 1) {
            pixel_count = 2;
        } else if (beambreakintake.getState() && pixel_count == 2) {
            pixel_count2 = 2;
        }
        if (!beambreakintake.getState() && pixel_count2 == 2) {
            intake_pathway.setPower(-1);
            intake_entrance.setPower(-1);
        }else if(lift_level > 0){
            intake_entrance.setPower(0);
            intake_pathway.setPower(0);
//            intake_pivotL.setPosition(0.07);
//            intake_pivotR.setPosition(0.07);
        }else if (pixel_count >=2){
            if(intake_timer2.seconds() > 1){
                if (gamepad1.right_trigger > 0.3) {
                    intake_entrance.setPower(0);
                    intake_pathway.setPower(0);
//                    intake_pivotL.setPosition(0.07);
//                    intake_pivotR.setPosition(0.07);
                } else if (gamepad1.left_trigger > 0.3) {
                    intake_timer.reset();
                    intake_entrance.setPower(0);
                    intake_pathway.setPower(-1);
//                    intake_pivotL.setPosition(0.07);
//                    intake_pivotR.setPosition(0.07);
                } else {
                    intake_entrance.setPower(0);
                    intake_pathway.setPower(0);
//                    intake_pivotL.setPosition(0.07);
//                    intake_pivotR.setPosition(0.07);

                }
            }else if (intake_timer2.seconds() > 0.5){
                intake_entrance.setPower(-1);
                intake_pathway.setPower(-1);

            } else{
                intake_entrance.setPower(-1);
                intake_pathway.setPower(1);
//                intake_pivotL.setPosition(0.25 + 0.04166 * 3);
//                intake_pivotR.setPosition(0.25 + 0.04166 * 3);
            }
        }else{
            intake_timer2.reset(); 
            if (gamepad1.right_trigger > 0.3) {
                intake_timer.reset();
                intake_entrance.setPower(intakeSpeed);
                intake_pathway.setPower(1);
//                intake_pivotL.setPosition(0.25 + 0.04166 * 3);
//                intake_pivotR.setPosition(0.25 + 0.04166 * 3);
            } else if (gamepad1.left_trigger > 0.3) {
                intake_timer.reset();
                intake_entrance.setPower(-1);
                intake_pathway.setPower(-1);
//                intake_pivotL.setPosition(0.25 + 0.04166 * 3);
//                intake_pivotR.setPosition(0.25 + 0.04166 * 3);
            } else {
                intake_entrance.setPower(0);
                intake_pathway.setPower(0);
                if (intake_timer.seconds() > 1) {
//                    intake_pivotL.setPosition(0.06);
//                    intake_pivotR.setPosition(0.06);
                }
            }
        }
        //gamepad 2 code
//        if(gamepad2.a){
//            intake_pivotL.setPosition(0.06);
//            intake_pivotR.setPosition(0.06);
//        }else if(gamepad2.x){
//            intake_pivotL.setPosition(0.083 + 0.04166*3 + 0.01388);
//            intake_pivotR.setPosition(0.083 + 0.04166*3 + 0.01388);
//        }else if(gamepad2.y){
//            intake_pivotL.setPosition(0.166 + 0.04166*3 + 0.01388);
//            intake_pivotR.setPosition(0.166 + 0.04166*3 + 0.01388);
//        }else if(gamepad2.b){
//            intake_pivotL.setPosition(0.25 + 0.04166*3 + 0.01);
//            intake_pivotR.setPosition(0.25 + 0.04166*3 + 0.01);
//        }

        //lift code
        if(gamepad2.right_bumper && !lift_pressed_up){
            if (lift_level == 0){
                lift_slow = true;
            }else {
                lift_slow = false;
            }
            lift_pressed_up = true;
            lift_level++;
            lift_level = Range.clip(lift_level, 0, 9);
        }else if(!gamepad2.right_bumper && lift_pressed_up){
            lift_pressed_up = false;

        }

        if(gamepad2.left_trigger > 0.2 && !lift_pressed_up2){
            if (lift_level == 0){
                lift_slow = true;
            }else {
                lift_slow = false;
            }
            lift_pressed_up2 = true;
            lift_level++;
            lift_level = Range.clip(lift_level, 0, 9);
        }else if(gamepad2.left_trigger < 0.2 && lift_pressed_up2){
            lift_pressed_up2 = false;

        }

        if(gamepad2.left_bumper && !lift_pressed_down){
            if(lift_level == 1) {
                lift_slow = true;
            }else{
                lift_slow = false;
            }
            lift_pressed_down = true;
            lift_level--;
            lift_level = Range.clip(lift_level, 0, 9);

        }else if(!gamepad2.left_bumper && lift_pressed_down){
            lift_pressed_down = false;
        }

        if(gamepad2.dpad_left){
            lift_slow = lift_level == 2 || lift_level == 1;

            lift_level = 0;
        }

        if(gamepad2.dpad_right){
            lift_slow = false;
            lift_level = 3;
        }

        if(gamepad2.dpad_up){
            lift_slow = false;
            lift_level = 8;
        }

        if(gamepad2.dpad_down){
            lift_slow = false;
            lift_level = 2;
        }


        if(lift_level == 0){
            pid = controller.calculate(lift_right.getCurrentPosition(),1);

        }else {
            pid = controller.calculate(lift_right.getCurrentPosition(),180 + lift_level*80 );

        }
        power = pid;


        if (lift_level == 0||lift_level == 1){
            if(lift_slow){
                if (lift_level == 0) {
                    lift_right.setPower(Range.clip(power, -0.25, 0.25));
                    lift_left.setPower(Range.clip(power,     -0.25, 0.25));
                }else {
                    lift_right.setPower(Range.clip(power, -0.4, 0.4));
                    lift_left.setPower(Range.clip(power, -0.4, 0.4));
                }
            }else {
                lift_right.setPower(Range.clip(power, -0.4, 0.4));
                lift_left.setPower(Range.clip(power, -0.4, 0.4));
            }
        }else{
            lift_right.setPower(Range.clip(power, -0.8, 0.8));
            lift_left.setPower(Range.clip(power, -0.8, 0.8));

        }
        controller.setPIDF(p, i, d, f);


        //manual testing
//        if (gamepad1.dpad_up){
//            lift_left.setPower(0.5);
//            lift_right.setPower(0.5);
//        }else if (gamepad1.dpad_down){
//            lift_left.setPower(-0.5);
//            lift_right.setPower(-0.5);
//        }else {
//            lift_left.setPower(0);
//            lift_right.setPower(0);
//        }



        //endgame

        if(gamepad2.start && gamepad2.back && gamepad2.right_trigger > 0.7 && gamepad2.left_trigger > 0.7){
            plane.setPosition(0.3);
        }

        //led color patterns
        //gren
//        if(gamepad2.a){
//            if(assigned == 0){
//                patterns.set(1,RevBlinkinLedDriver.BlinkinPattern.GREEN);
//                assigned = 1;
//            }else if(assigned == 1){
//                patterns.set(2,RevBlinkinLedDriver.BlinkinPattern.GREEN);
//                assigned = 2;
//            }else if(assigned == 2){
//                patterns = new Vector<RevBlinkinLedDriver.BlinkinPattern>(2,1);
//                patterns.set(1,RevBlinkinLedDriver.BlinkinPattern.GREEN);
//                assigned = 0;
//            }
//        }else if(gamepad2.x){//purpl
//            if(assigned == 0){
//                patterns.set(1,RevBlinkinLedDriver.BlinkinPattern.VIOLET);
//                assigned = 1;
//            }else if(assigned == 1){
//                patterns.set(2,RevBlinkinLedDriver.BlinkinPattern.VIOLET);
//                assigned = 2;
//            }else if(assigned == 2){
//                patterns = new Vector<RevBlinkinLedDriver.BlinkinPattern>(2,1);
//                patterns.set(1,RevBlinkinLedDriver.BlinkinPattern.VIOLET);
//                assigned = 0;
//            }
//
//        }else if(gamepad2.y){//yellow
//            if(assigned == 0){
//                patterns.set(1,RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//                assigned = 1;
//            }else if(assigned == 1){
//                patterns.set(2,RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//                assigned = 2;
//            }else if(assigned == 2){
//                patterns = new Vector<RevBlinkinLedDriver.BlinkinPattern>(2,0);
//                patterns.set(1,RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//                assigned = 0;
//            }
//
//        }else if(gamepad2.b){//white
//            if(assigned == 0){
//                patterns.set(1,RevBlinkinLedDriver.BlinkinPattern.WHITE);
//                assigned = 1;
//            }else if(assigned == 1){
//                patterns.set(2,RevBlinkinLedDriver.BlinkinPattern.WHITE);
//                assigned = 2;
//            }else if(assigned == 2){
//                patterns = new Vector<RevBlinkinLedDriver.BlinkinPattern>(2,0);
//                patterns.set(1,RevBlinkinLedDriver.BlinkinPattern.WHITE);
//                assigned = 0;
//            }
//
//        }
//
//        if(patterns.size() > 1){
//            if (pattern_stage == 0){
//                led_timer.reset();
//                pattern_stage = 1;
//                ledDriver.setPattern(patterns.elementAt(1));
//            }else if(pattern_stage == 1 && led_timer.seconds() > 0.5){
//                led_timer.reset();
//                pattern_stage = 2;
//                ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
//            }else if(pattern_stage == 2 && led_timer.seconds() > 0.25){
//                led_timer.reset();
//                pattern_stage = 3;
//                ledDriver.setPattern(patterns.elementAt(2));
//            }else if(pattern_stage == 3 && led_timer.seconds() > 0.5){
//                led_timer.reset();
//                pattern_stage = 4;
//                ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
//            }else if(pattern_stage == 4 && led_timer.seconds() > 1){
//                led_timer.reset();
//                pattern_stage = 0;
//            }
//
//        }else {
//            pattern_stage = 0;
//            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
//        }
//        if (!beambreak.getState()){
//            pattern_stage = 0;
//            patterns = new Vector<RevBlinkinLedDriver.BlinkinPattern>(2,0);
//            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
//        }
    
        if (gamepad1.dpad_up && !intake_button_test1){
            intake_button_test1 = true;
            intake_pivotL.setPosition(intake_pivotL.getPosition() + 0.01);
            intake_pivotR.setPosition(intake_pivotR.getPosition() + 0.01);
        }else if (!gamepad1.dpad_up && intake_button_test1){
            intake_button_test1 = false;
        }

        if (gamepad1.dpad_down && !intake_button_test2){
            intake_button_test2 = true;
            intake_pivotL.setPosition(intake_pivotL.getPosition() - 0.01);
            intake_pivotR.setPosition(intake_pivotR.getPosition() - 0.01);
        }else if (!gamepad1.dpad_down && intake_button_test2){
            intake_button_test2 = false;
        }

        if (gamepad1.dpad_left && !intake_button_test3){
            intake_button_test3 = true;
            intakeSpeed+=0.05;
        }else if (!gamepad1.dpad_left && intake_button_test3){
            intake_button_test3 = false;
        }

        if (gamepad1.dpad_right && !intake_button_test4){
            intake_button_test4 = true;
            intakeSpeed-=0.05;
        }else if (!gamepad1.dpad_right && intake_button_test4){
            intake_button_test4 = false;
        }




        if(lift_level == 0){
            telemetrydash.addData("target", 0);
        }else {
            telemetrydash.addData("target", 150 + lift_level*100);
        }
        telemetrydash.addData("lift", lift_right.getCurrentPosition());

        telemetry.addData("p", p);
        telemetry.addData("i", i);
        telemetry.addData("d", d);
        telemetry.addData("pixels count", pixel_count);
        telemetry.addData("pixels count 2", pixel_count2);
        if(!beambreakintake.getState()){
            telemetry.addData("beambreak intake ", "broken");
        }else {
            telemetry.addData("beambreak intake ", "closed");
        }
        if(!beambreakouttake.getState()){
            telemetry.addData("beambreak outtake ", "broken");
        }else {
            telemetry.addData("beambreak outtake ", "closed");
        }
//        telemetry.addData("cam1 ", visionPortal1.getCameraState());
        //telemetry.addData("cam2 ", visionPortal2.getCameraState());
        myLocalizer.update();
        telemetry.addData("pose",myLocalizer.getPoseEstimate());
        telemetry.addData("yerror", yError);
        telemetry.addData("xerror", xError);
        telemetry.addData("heading error", headingError);
        telemetry.update();
        telemetrydash.update();

        myLocalizer.update();
        botpose = myLocalizer.getPoseEstimate();
    }
}
