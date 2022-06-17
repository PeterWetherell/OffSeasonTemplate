package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
@TeleOp
public class trackWidthTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        double ticksPerRotation = 8192.0;
        double wheelRadius = 0.6889764;
        double ticksToInches = (wheelRadius * Math.PI * 2.0)/ticksPerRotation;
        telemetry.addData("Assumption 1","the robot does move left/right or forward/backward");
        telemetry.addData("Assumption 2","the robot is only rotating");
        telemetry.update();
        waitForStart();
        for (int i = 0; i < 4; i ++){
            drive.motors.get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.motors.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double lastAngle = drive.imu.getAngularOrientation().firstAngle;
        double currentCumAngle = 0;
        while (opModeIsActive()){

            double turn = gamepad1.right_stick_x * 0.35;
            double p1 = turn;
            double p2 = turn;
            double p3 = -1 * turn;
            double p4 = -1 * turn;
            drive.setMotorPowers(p1, p2, p3, p4);

            double currentAngle = drive.imu.getAngularOrientation().firstAngle;
            double deltaAngle = currentAngle - lastAngle;
            while (deltaAngle >= Math.PI){
                deltaAngle -= Math.PI * 2;
            }
            while (deltaAngle <= -Math.PI){
                deltaAngle += Math.PI * 2;
            }
            currentCumAngle += deltaAngle;
            lastAngle = currentAngle;
            double right = drive.rightFront.getCurrentPosition() * ticksToInches * drive.localizer.encoders[0].scaleFactor;
            double left = drive.leftFront.getCurrentPosition() * ticksToInches * drive.localizer.encoders[1].scaleFactor;
            double back = drive.rightBack.getCurrentPosition() * ticksToInches * drive.localizer.encoders[2].scaleFactor;


            double trackWidth = 0;
            if (currentCumAngle != 0) {
                trackWidth = (right - left) / currentCumAngle;
            }

            telemetry.addData("Cumulative Angle",Math.toDegrees(currentCumAngle));
            telemetry.addData("Track Width",trackWidth);
            telemetry.addData("Right Odo Y",(-1 * right / currentCumAngle));
            telemetry.addData("Left Odo Y", (-1 * left / currentCumAngle));
            telemetry.addData("Back Odo X",back/currentCumAngle);
            telemetry.addData("Right",right);
            telemetry.addData("Left",left);
            telemetry.addData("Back",back);
            telemetry.update();
        }
    }
}