package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Pose2d;
import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Trajectory;

@Autonomous
public class WearhouseAutoRed  extends LinearOpMode {
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Assumption 1","place the robot facing forward in the bottom right corner of a 4 by 4 field");
        telemetry.addData("Assumption 2","center the robot on the corner of all 4 tiles");
        telemetry.update();
        waitForStart();
        for (int i = 0; i < 4; i ++){
            drive.motors.get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.motors.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.motors.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        drive.localizer.x = 12;
        drive.localizer.y = -65.25;
        int i = 0;
        while (opModeIsActive()) {
            double angle = Math.toRadians(10) * (i % 3);
            Trajectory trajectory1 = new Trajectory(new Pose2d(12, -65.25, 0, 0, 8, 0.3), true)
                    .addLine(new Pose2d(32, -65.25, 0, 0, 6, 0.8))
                    .addLine(new Pose2d(34 + Math.cos(angle) * 8.0, -65.25 + Math.sin(angle) * 8.0, 0, 0, 4, 0.8))
                    .end();
            while (opModeIsActive() && trajectory1.points.size() >= 1) {
                drive.target = trajectory1.points.get(0);
                drive.update();
                drive.pinMotorPowers(trajectory1.update(drive.currentPose, drive.relCurrentVel));
            }
            waitMillis(500);

            Trajectory trajectory2 = new Trajectory(new Pose2d(drive.currentPose.x, drive.currentPose.y, Math.toRadians(180), 0, 8, 0.3), true)
                    .addLine(new Pose2d(40, -65.25,0,Math.toRadians(180),4,1))
                    .addLine(new Pose2d(12, -65.25,0,Math.toRadians(180),8,1))
                    .end();
            while (opModeIsActive() && trajectory2.points.size() >= 1) {
                drive.target = trajectory2.points.get(0);
                drive.update();
                drive.pinMotorPowers(trajectory2.update(drive.currentPose, drive.relCurrentVel));
            }
            waitMillis(500);
            i ++;
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