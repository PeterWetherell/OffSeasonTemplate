package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

@TeleOp
public class headingErrorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        for (int i = 0; i < 4; i ++){
            drive.motors.get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.motors.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        drive.setPose(0,-12,Math.toRadians(0));
        while (opModeIsActive()){
            drive.update();

            double turn = gamepad1.right_stick_x * 0.35;
            double forward = gamepad1.left_stick_y * -0.45;
            double left = gamepad1.left_stick_x * 0.55;
            double p1 = forward - left + turn;
            double p2 = forward + left  + turn;
            double p3 = forward - left  - turn;
            double p4 = forward + left  - turn;
            drive.pinMotorPowers(p1, p2, p3, p4);

            double currentAngle = drive.imu.getAngularOrientation().firstAngle;

            double headingError = (currentAngle - drive.currentPose.heading);
            while (headingError >= Math.PI){
                headingError -= 2 * Math.PI;
            }
            while (headingError <= -Math.PI){
                headingError += 2 * Math.PI;
            }
            telemetry.addData("currentHeadingError", Math.toDegrees(headingError));
            telemetry.update();
        }
    }
}
