package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
@TeleOp
public class LocalizationTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPose(0,0,0);
        waitForStart();

//        for (int i = 0; i < 4; i ++){
//            drive.motors.get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            drive.motors.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            drive.motors.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }

        for (int i = 0; i < 3; i ++){
            drive.motors.get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.motors.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            drive.motors.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        while (opModeIsActive()){
            double forward = gamepad1.left_stick_y * -1;
            double left = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            double p1 = forward+left+turn;
            double p2 = forward-left+turn;
            double p3 = forward+left-turn;
            double p4 = forward-left-turn;
            drive.setMotorPowers(p1, p2, p3, p4);
//            drive.update();
        }
    }
}
