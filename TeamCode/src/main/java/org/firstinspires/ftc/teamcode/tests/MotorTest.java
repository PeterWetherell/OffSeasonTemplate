package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

@TeleOp(group = "Test")
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        double[] motorPower = new double[drive.motors.size()];
        for (int i = 0; i < drive.motors.size(); i ++){
            motorPower[i] = 0;
        }
        int motorIndex = 0;
        boolean lastX = false;
        boolean lastY = false;
        double numLoops = 0;
        double totalTime = 0;
        while (!isStopRequested()) {
            numLoops ++;
            if (gamepad1.right_trigger >= 0.5){
                motorPower[motorIndex] += 0.001;
            }
            if (gamepad1.left_trigger >= 0.5){
                motorPower[motorIndex] -= 0.001;
            }
            motorPower[motorIndex] = Math.min(1.0,Math.max(-1.0,motorPower[motorIndex]));

            if (gamepad1.a){
                motorPower[motorIndex] = 0;
            }

            long start = System.nanoTime();
            drive.motors.get(motorIndex).setPower(motorPower[motorIndex]);
            double elapsedTime = (System.nanoTime()-start)/1000000000.0;
            totalTime += elapsedTime;

            boolean x = gamepad1.x;
            if (x && !lastX){
                motorIndex += drive.motors.size() - 1;
            }
            lastX = x;
            boolean y = gamepad1.y;
            if (y && !lastY){
                motorIndex += 1;
            }
            lastY = y;
            motorIndex = motorIndex % drive.motors.size();

            telemetry.addData("motorNum", motorIndex);
            telemetry.addData("motorPower", motorPower[motorIndex]);
            telemetry.addData("averageMotorTime", totalTime/numLoops);
            telemetry.update();
        }
    }
}
