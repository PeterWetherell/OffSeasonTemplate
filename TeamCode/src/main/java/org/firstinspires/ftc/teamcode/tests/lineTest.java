package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Pose2d;
import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Trajectory;

import java.util.ArrayList;

@TeleOp
public class lineTest extends LinearOpMode {
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Assumption 1","place the robot facing forward in the bottom right corner of a 4 by 4 field");
        telemetry.addData("Assumption 2","center the robot on the tile");
        telemetry.update();
        waitForStart();
        for (int i = 0; i < 4; i ++){
            drive.motors.get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.motors.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.motors.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        drive.setPose(0,-12,Math.toRadians(0));
        for (int i = 0; i < 4; i ++) {
            Trajectory trajectory1 = new Trajectory(new Pose2d(0,-12,0,0.2), true)
                    .addLine(new Pose2d(60,-12,0,0,18,1))
                    .end();
            while (opModeIsActive() && trajectory1.points.size() >= 1){
                drive.target = trajectory1.points.get(0);
                drive.update();
                drive.pinMotorPowers(trajectory1.update(drive.currentPose, drive.relCurrentVel));
            }
            waitMillis(2000);
            Trajectory trajectory2 = new Trajectory(new Pose2d(60,-12,0,0.2), true)
                    .addLine(new Pose2d(0,-12,0,Math.toRadians(180),18,1))
                    .end();
            while (opModeIsActive() && trajectory2.points.size() >= 1){
                drive.target = trajectory2.points.get(0);
                drive.update();
                drive.pinMotorPowers(trajectory2.update(drive.currentPose, drive.relCurrentVel));
            }
            waitMillis(2000);
        }
    }
    public void waitMillis(long time){
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < time){
            drive.pinMotorPowers(0,0,0,0);
            drive.update();
        }
    }
}
