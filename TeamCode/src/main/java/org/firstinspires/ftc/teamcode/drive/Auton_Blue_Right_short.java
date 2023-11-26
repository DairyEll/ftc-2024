package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Auton_Blue_Right_short extends LinearOpMode {
    private PIDFController controller;
    double p = 0.005;
    double i = 0.1;
    double d = 0.00055;
    double f = 0.0002;
    Pose2d converge_end = new Pose2d();
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        controller = new PIDFController(p, i, d, f);
        controller.setPIDF(p,i,d, f);
        Pose2d startPose = new Pose2d(-63.75, 4.75, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        Trajectory left_spike = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-32.00, 3, Math.toRadians(90.00)))
                .build();
        Trajectory left_deposit = drive.trajectoryBuilder(left_spike.end())
                .lineToLinearHeading(new Pose2d(-43.00, 48.00, Math.toRadians(90.00)))
                .build();
        TrajectorySequence left_park = drive.trajectorySequenceBuilder(left_deposit.end()/*converge_end*/)
                .lineToLinearHeading(new Pose2d(-60, 44.00, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-60, 61, Math.toRadians(0)))
                .build();

        waitForStart();
        drive.followTrajectory(left_spike);
        sleep(100);
        drive.deposit.setPower(1);
        sleep(400);
        drive.deposit.setPower(0);
        drive.followTrajectory(left_deposit);
        sleep(100);
        drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),450 ), -0.5, 0.5));
        drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),450 ), -0.5, 0.5));
        while(drive.lift_right.getPower() > 0.1){
            drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),450 ), -0.5, 0.5));
            drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),450 ), -0.5, 0.5));
        }
        drive.lift_right.setPower(0.01);
        drive.lift_left.setPower(0.01);
        sleep(100);
        drive.deposit.setPower(1);
        sleep(200);
        drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),550 ));
        drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),550 ));
        while(drive.lift_right.getPower() > 0.1){
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),550 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),550 ));
        }
        drive.lift_right.setPower(0.01);
        drive.lift_left.setPower(0.01);
        sleep(100);
        drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
        drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
        drive.deposit.setPower(0);
        while(drive.lift_right.getPower() > 0.1){
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),0 ));
        }
        drive.lift_right.setPower(0.01);
        drive.lift_left.setPower(0.01);
        sleep(200);
        drive.followTrajectorySequence(left_park);





    }
}
