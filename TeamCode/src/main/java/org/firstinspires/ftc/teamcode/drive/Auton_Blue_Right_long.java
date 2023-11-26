package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Auton_Blue_Right_long extends LinearOpMode {
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
        Pose2d startPose = new Pose2d(-63.75, -40.25, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        Trajectory left_spike = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-32.00, -35.25, Math.toRadians(90.00)))
                .build();
        Trajectory left_stack = drive.trajectoryBuilder(left_spike.end())
                .lineToLinearHeading(new Pose2d(-38.00, -57.75, Math.toRadians(90.00)))
                .build();
        TrajectorySequence left_deposit1 = drive.trajectorySequenceBuilder(left_stack.end())
                .lineToLinearHeading(new Pose2d(-12.00, -56.7, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-12.00, 31.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-43.00, 48.00, Math.toRadians(90.00)))
                .build();
        TrajectorySequence stack1 = drive.trajectorySequenceBuilder(left_deposit1.end()/*converge_end*/)
                .lineToLinearHeading(new Pose2d(-12.00, 31.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-12.00, -56.70, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-24.00, -56.75, Math.toRadians(90.00)))
                .build();

        waitForStart();
        drive.followTrajectory(left_spike);
        sleep(100);
        drive.deposit.setPower(1);
        sleep(400);
        drive.deposit.setPower(0);
        drive.followTrajectory(left_stack);
        sleep(100);
        drive.intake_entrance.setPower(1);
        drive.intake_pathway.setPower(1);
        drive.intake_pivotL.setPosition(0.65);
        drive.intake_pivotR.setPosition(0.65);
        sleep(1000);
        drive.intake_pivotL.setPosition(0.7);
        drive.intake_pivotR.setPosition(0.7);
        drive.intake_entrance.setPower(0);
        drive.intake_pathway.setPower(0);
        drive.followTrajectorySequence(left_deposit1);
        sleep(100);
        drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),450 ), -0.5, 0.5));
        drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),450 ), -0.5, 0.5));
        while(drive.lift_right.getPower() > 0.1){
            drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),450 ), -0.5, 0.5));
            drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),450 ), -0.5, 0.5));
        }
        sleep(100);
        drive.deposit.setPower(1);
        sleep(200);
        drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),550 ));
        drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),550 ));
        while(drive.lift_right.getPower() > 0.1){
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),550 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),550 ));
        }
        sleep(100);
        drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
        drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
        drive.deposit.setPower(0);

        while(drive.lift_right.getPower() > 0.1){
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
            drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
        }

        sleep(200);
        drive.deposit.setPower(1);
        sleep(200);
        drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),550 ));
        drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),550 ));
        while(drive.lift_right.getPower() > 0.1){
            drive.lift_left.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
             drive.lift_right.setPower(controller.calculate(drive.lift_right.getCurrentPosition(),450 ));
        }

        sleep(200);
        drive.deposit.setPower(0);
        drive.lift_left.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),0 ), -0.25, 0.25));
        drive.lift_right.setPower(Range.clip(controller.calculate(drive.lift_right.getCurrentPosition(),0 ), -0.25, 0.25));
        sleep(500);
        drive.followTrajectorySequence(stack1);





    }
}
