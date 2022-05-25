package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
@TeleOp
public class LocalizationTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPose(0,0,0);
        waitForStart();
        while (opModeIsActive()){
            double forward = gamepad1.left_stick_y * -0.4;
            double left = gamepad1.left_stick_x * 0.6;
            double turn = gamepad1.right_stick_x * 0.35;
            double p1 = forward+left+turn;
            double p2 = forward-left+turn;
            double p3 = forward+left-turn;
            double p4 = forward-left-turn;
            drive.pinMotorPowers(p1, p2, p3, p4);
            drive.update();
        }
    }
}
