package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Pose2d;
import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Trajectory;
import org.firstinspires.ftc.teamcode.TrajectoryPeice;

import java.util.ArrayList;

@TeleOp
public class movementTest extends LinearOpMode {
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Assumption 1","place the robot facing forward in the bottom right corner of a 4 by 4 field");
        telemetry.addData("Assumption 2","center the robot on the corner tile");
        telemetry.update();
        waitForStart();
        for (int i = 0; i < 4; i ++){
            drive.motors.get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.motors.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.motors.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        drive.setPose(-12,-12,Math.toRadians(0));
        Trajectory trajectory1 = new Trajectory(new TrajectoryPeice(new Pose2d(0,0,0),0,0.3), true)
                .addLine(new TrajectoryPeice(new Pose2d(48,0),0,1))
                .addLine(new TrajectoryPeice(new Pose2d(48,48),0,1))
                .addLine(new TrajectoryPeice(new Pose2d(0,48),0,1))
                .addLine(new TrajectoryPeice(new Pose2d(0,0,0),0,1))
                .end();
        drive.followTrajectory(this,trajectory1);
        waitMillis(2000);
    }
    public void waitMillis(long time){
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < time){
            drive.pinMotorPowers(0,0,0,0);
            drive.update();
        }
    }
}