package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Timer;
import java.util.concurrent.TimeUnit;

@TeleOp(name="intake_entrance outtake", group="Iterative OpMode")
public class drivecode extends OpMode {
    final double DESIRED_DISTANCE = 24.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.075 ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.009 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.09  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 1;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 1;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotorEx front_left;
    private DcMotorEx front_right;
    private DcMotorEx back_left;
    private DcMotorEx back_right;

    private DcMotorEx lift_left;
    private DcMotorEx lift_right;

    private DcMotorEx intake_entrance;
    private DcMotorEx intake_pathway;

    private Servo intake_pivotL;
    private Servo intake_pivotR;

    private CRServo deposit;

    private Servo plane;

    public RevBlinkinLedDriver ledDriver;

    public IMU imufordrive;

    private ElapsedTime deposittimer = new ElapsedTime();

    private boolean depositing = false;

    private AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
            .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
            .build();
    private VisionPortal visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(aprilTag)
            .setCameraResolution(new Size(640,480))
            .build();
    private AprilTagDetection desiredTag = null;

    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turn            = 0;

    int lift_level = 0;
    int board_pos;

    boolean position_change = false;
    boolean lift_pressed_up = false;
    boolean lift_pressed_down = false;
    MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(10,10), 10));

    RevBlinkinLedDriver.BlinkinPattern[] patterns = new RevBlinkinLedDriver.BlinkinPattern[2];

    int assigned = 0;




    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(10,10), 10));
        front_left = hardwareMap.get(DcMotorEx.class, "frontleft");
        back_left = hardwareMap.get(DcMotorEx.class, "backleft");
        front_right = hardwareMap.get(DcMotorEx.class, "frontright");
        back_right = hardwareMap.get(DcMotorEx.class, "backright");
        front_left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setVelocity(0);
        back_left.setVelocity(0);
        front_right.setVelocity(0);
        back_right.setVelocity(0);

        lift_right = hardwareMap.get(DcMotorEx.class, "rightlift");
        lift_left = hardwareMap.get(DcMotorEx.class, "leftlift");
        lift_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift_left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_left.setDirection(DcMotorSimple.Direction.REVERSE);
        lift_right.setVelocity(0);
        lift_left.setVelocity(0);

        intake_entrance = hardwareMap.get(DcMotorEx.class, "intakeentrance");
        intake_entrance.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake_entrance.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_pathway = hardwareMap.get(DcMotorEx.class, "intakepathway");
        intake_pathway.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake_pathway.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake_pivotL = hardwareMap.get(Servo.class, "intakepivotL");
        intake_pivotR = hardwareMap.get(Servo.class, "intakepivotR");

        deposit = hardwareMap.get(CRServo.class, "deposit");

        plane = hardwareMap.get(Servo.class, "plane");


        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "ledDriver");

        imufordrive = hardwareMap.get(IMU.class, "imu");

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.PLACEHOLDER,
//                RevHubOrientationOnRobot.UsbFacingDirection.PLACEHOLDER));
// Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imufordrive.initialize(parameters);

        imufordrive.resetYaw();
        if(visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){
            telemetry.addData("Camera", "starting");
            telemetry.update();
        }else if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING && exposureControl.getExposure(TimeUnit.MILLISECONDS) != 20 && gainControl.getGain()!=gainControl.getMaxGain()){
            exposureControl.setMode(ExposureControl.Mode.ContinuousAuto);
            exposureControl.setExposure(20, TimeUnit.MILLISECONDS);

            gainControl.setGain(gainControl.getMaxGain());
            telemetry.addData("Camera", "setting up");
            telemetry.update();

        }else if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING && exposureControl.getExposure(TimeUnit.MILLISECONDS) == 20 && gainControl.getGain()==gainControl.getMaxGain()){
            telemetry.addData("Camera", "ready");
            telemetry.update();
        }else{
            telemetry.addData("Camera", "shouldnt be like this :sob:");
            telemetry.update();
        }





    }

    /**
     *
     */
    @Override
    public void loop() {
        mecanumDrive.updatePoseEstimate();
        Pose2d botpose = mecanumDrive.pose;
        //gamepad1 code


        //drivetrain code
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    (detection.id == 1)  ){
                targetFound = true;
                desiredTag = detection;

                break;  // don't look any further.
            }else if ((detection.metadata != null) &&
                    (detection.id == 2)  ){
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            } else if ((detection.metadata != null) &&
                    (detection.id == 3)  ){
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            }
            else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                board_pos = 0;
                position_change = false;


            }
        }

        if (targetFound) {
            telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);

        }

        double botHeading = ((imufordrive.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))/* +autonomous1.anglestored+ autonomous2.anglestored*/);
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
        if(botpose.position.x < 0){
            board_pos = 0;
        }


        if (gamepad1.left_bumper && botpose.position.x > 12) {
            double xError;
            double headingError;
            double yawError;

            if(lift_level%2 == 0){
                board_pos = Range.clip(board_pos, -3, 3);
                yawError = 36 - botpose.position.y - 3.5 * board_pos;
            }else{
                board_pos = Range.clip(board_pos, -2, 3);
                yawError = (36 + 3.5/2) - botpose.position.y - 3.5 * board_pos;
            }

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            if(targetFound) {
                 xError = (desiredTag.ftcPose.y - DESIRED_DISTANCE);
                 headingError = desiredTag.ftcPose.bearing;
                 if(xError < 1 && yawError < 1){
                     //relocalize using apriltags
                 }
            }else{
                xError = 36-botpose.position.x;
                headingError = 90 - botpose.heading.log();
            }


            if(!position_change && gamepad1.left_stick_x>0.3){
                position_change = true;
                board_pos -=1;
            }else if(!position_change && gamepad1.left_stick_x<-0.3){
                position_change = true;
                board_pos +=1;
            } else if (position_change && (gamepad1.left_stick_x < 0.3 || gamepad1.left_stick_x > -0.3)) {
                position_change = false;
            }




            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = -Range.clip(xError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            double leftFrontPower    =  drive -strafe -turn;
            double rightFrontPower   =  drive +strafe +turn;
            double leftBackPower     =  drive +strafe -turn;
            double rightBackPower    =  drive -strafe +turn;

            front_left.setVelocity(2500 * leftFrontPower);
            back_left.setVelocity(2500 * leftBackPower);
            front_right.setVelocity(2500 * rightFrontPower);
            back_right.setVelocity(2500 * rightBackPower);

            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }else {

            front_left.setVelocity(2500 * frontLeftPower);
            back_left.setVelocity(2500 * backLeftPower);
            front_right.setVelocity(2500 * frontRightPower);
            back_right.setVelocity(2500 * backRightPower);
        }

        //deposit code
        if(gamepad1.a && !depositing){
            deposittimer.reset();
            depositing = true;
            deposit.setPower(1);

        }else if(deposittimer.seconds() > 0.3&& depositing){
            deposittimer.reset();
            depositing = false;
            deposit.setPower(0);
        }
        else if(!depositing) {
            deposit.setPower(0);
        }

        //intake code
        if(gamepad1.right_trigger>0.3){
            intake_entrance.setVelocity(2500);
            intake_pathway.setVelocity(2500);
        } else if (gamepad1.left_stick_button) {
            intake_entrance.setVelocity(2500);
            intake_pathway.setVelocity(2500);
        }else {
            intake_entrance.setVelocity(0);
            intake_pathway.setVelocity(0);
        }//gamepad 2 code
//        if(gamepad2.a){
//            intake_pivotL.setPosition(0);
//            intake_pivotR.setPosition(0);
//        }else if(gamepad2.x){
//            intake_pivotL.setPosition(0.11111111111);
//            intake_pivotR.setPosition(0.11111111111);
//        }else if(gamepad2.y){
//            intake_pivotL.setPosition(0.22222222222);
//            intake_pivotR.setPosition(0.22222222222);
//        }else if(gamepad2.b){
//            intake_pivotL.setPosition(0.33333333333);
//            intake_pivotR.setPosition(0.33333333333);
//        }

        //lift code
        if(gamepad2.dpad_up && !lift_pressed_up){
            lift_pressed_up = true;
            lift_level++;
            lift_level = Range.clip(lift_level, 0, 7);
        }else if(!gamepad2.dpad_up && lift_pressed_up){
            lift_pressed_up = false;
        }

        if(gamepad2.dpad_down && !lift_pressed_down){
            lift_pressed_up = true;
            lift_level--;
            lift_level = Range.clip(lift_level, 0, 7);
        }else if(!gamepad2.dpad_down && lift_pressed_down){
            lift_pressed_up = false;
        }

        if(gamepad2.dpad_left){
            lift_level = 0;
        }

        if(gamepad2.dpad_right){
            lift_level = 3;
        }

        if(lift_level == 0){
            setLift(0);
        }else {
            setLift(500 + lift_level*300);
        }

        //endgame

        if(gamepad2.start && gamepad2.back && gamepad2.right_trigger > 0.7 && gamepad2.left_trigger > 0.7){
            plane.setPosition(0.3);
        }

        //led color patterns
        //gren
        if(gamepad2.a){
            if(assigned == 0){
                patterns[0] = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                assigned = 1;
            }else if(assigned == 1){
                patterns[1] = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                assigned = 2;
            }
        }else if(gamepad2.x){//purpl

        }else if(gamepad2.y){//demian

        }else if(gamepad2.b){//max laine

        }









        telemetry.update();
    }

    public void setLift(int position){
        lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_left.setTargetPosition(position);
        lift_right.setTargetPosition(position);
        lift_left.setVelocity(2500);
        lift_left.setVelocity(2500);

    }
}
