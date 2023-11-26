package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="encoder test", group="Iterative OpMode")
public class encoder_test extends OpMode {
    private DcMotorEx front_left;
    private DcMotorEx front_right;
    private DcMotorEx back_left;
    private DcMotorEx back_right;

    private PIDController controller;

    private DcMotorEx lift_left;
    private DcMotorEx lift_right;

    private DcMotorEx intake_entrance;
    private DcMotorEx intake_pathway;



    private Servo intake_pivotL;
    private Servo intake_pivotR;

    private CRServo deposit;

    private Servo plane;

    public RevBlinkinLedDriver ledDriver;

    private DigitalChannel beambreakintake;
    private DigitalChannel beambreakouttake;
    private IMU imufordrive;

    @Override
    public void init() {
        front_left = hardwareMap.get(DcMotorEx.class, "frontleft");
        back_left = hardwareMap.get(DcMotorEx.class, "backleft");
        front_right = hardwareMap.get(DcMotorEx.class, "frontright");
        back_right = hardwareMap.get(DcMotorEx.class, "backright");
        front_left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        front_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        front_left.setDirection(DcMotorEx.Direction.REVERSE);
        back_left.setDirection(DcMotorEx.Direction.REVERSE);
        front_left.setVelocity(0);
        back_left.setVelocity(0);
        front_right.setVelocity(0);
        back_right.setVelocity(0);


        lift_right = hardwareMap.get(DcMotorEx.class, "rightlift");
        lift_left = hardwareMap.get(DcMotorEx.class, "leftlift");
        lift_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift_left.setDirection(DcMotorEx.Direction.REVERSE);
        lift_right.setVelocity(0);
        lift_left.setVelocity(0);


        intake_entrance = hardwareMap.get(DcMotorEx.class, "intakeentrance");
        intake_entrance.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake_entrance.setDirection(DcMotorEx.Direction.REVERSE);
        intake_entrance.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake_pathway = hardwareMap.get(DcMotorEx.class, "intakepathway");
        intake_pathway.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake_pathway.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake_pathway.setDirection(DcMotorEx.Direction.REVERSE);

        intake_pivotL = hardwareMap.get(Servo.class, "intakepivotL");
        intake_pivotL.setDirection(Servo.Direction.REVERSE);
        intake_pivotR = hardwareMap.get(Servo.class, "intakepivotR");
        intake_pivotL.setPosition(0.06);
        intake_pivotL.setPosition(0.06);

        deposit = hardwareMap.get(CRServo.class, "deposit");

        plane = hardwareMap.get(Servo.class, "plane");



        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "ledDriver");

        imufordrive = hardwareMap.get(IMU.class, "imu");

        beambreakintake = hardwareMap.get(DigitalChannel.class, "beambreakintake");
        beambreakintake.setMode(DigitalChannel.Mode.INPUT);
        beambreakouttake = hardwareMap.get(DigitalChannel.class, "beambreakouttake");
        beambreakouttake.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {
        telemetry.addData("left front", front_left.getCurrentPosition());
        telemetry.addData("left back", back_left.getCurrentPosition());
        telemetry.addData("right front", front_right.getCurrentPosition());
        telemetry.addData("right back", back_right.getCurrentPosition());
        telemetry.addData("left lift", lift_left.getCurrentPosition());
        telemetry.addData("right lift", back_left.getCurrentPosition());
        telemetry.addData("intake pathway", intake_pathway.getCurrentPosition());
        telemetry.addData("right entrance", intake_entrance.getCurrentPosition());

        telemetry.update();
    }
}
