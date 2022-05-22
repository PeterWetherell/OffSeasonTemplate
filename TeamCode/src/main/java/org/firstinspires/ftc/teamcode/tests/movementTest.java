package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Pose2d;
import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

@TeleOp
public class movementTest extends LinearOpMode {
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
        drive.localizer.x = -12;
        drive.localizer.y = -12;
        while (opModeIsActive()){
            moveToPoint(new Pose2d(60,-12),0.5, 4);
            moveToPoint(new Pose2d(60,60),0.5, 4);
            moveToPoint(new Pose2d(-12,60),0.5, 4);
            moveToPoint(new Pose2d(-12,-12),0.5, 4);
        }
    }
    public void moveToPoint(Pose2d target, double speed, double minError){
        double error = Math.sqrt(Math.pow(drive.currentPose.x - target.x,2) + Math.pow(drive.currentPose.y - target.y,2));
        while (opModeIsActive() && error >= minError){
            error = Math.sqrt(Math.pow(drive.currentPose.x - target.x,2) + Math.pow(drive.currentPose.y - target.y,2));

            double targetAngle = Math.atan2(target.y - drive.currentPose.y,target.x - drive.currentPose.x);

            double errorX = target.x - drive.currentPose.x;
            double errorY = target.y - drive.currentPose.y;
            double errorHeading = targetAngle - drive.currentPose.getHeading();
            while (errorHeading >= Math.PI){
                errorHeading -= 2 * Math.PI;
            }
            while (errorHeading <= -Math.PI){
                errorHeading += 2 * Math.PI;
            }

            double relErrorX = error * Math.cos(errorHeading);
            double relErrorY = error * Math.sin(errorHeading);
            double velX = speed * relErrorX / (Math.abs(relErrorX) + Math.abs(relErrorY));
            double velY = speed * relErrorY / (Math.abs(relErrorX) + Math.abs(relErrorY));
            double turn = Math.toDegrees(errorHeading)/8.0 * speed;

            double p1 = velX-velY-turn;
            double p2 = velX+velY-turn;
            double p3 = velX-velY+turn;
            double p4 = velX+velY+turn;
            drive.pinMotorPowers(p1, p2, p3, p4);

            drive.update();
        }
    }
}
