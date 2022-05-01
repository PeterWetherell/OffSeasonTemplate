package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

@TeleOp(group = "Test")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        double[] servoPos = new double[drive.servos.size()];
        for (int i = 0; i < drive.servos.size(); i ++){
            servoPos[i] = 0.5;
        }
        int servoIndex = 0;
        boolean lastX = false;
        boolean lastY = false;
        double numLoops = 0;
        double totalTime = 0;
        while (!isStopRequested()) {
            numLoops ++;
            if (gamepad1.a){
                servoPos[servoIndex] += 0.001;
            }
            if (gamepad1.b){
                servoPos[servoIndex] -= 0.001;
            }
            servoPos[servoIndex] = Math.min(1.0,Math.max(0,servoPos[servoIndex]));

            long start = System.nanoTime();
            drive.servos.get(servoIndex).setPosition(servoPos[servoIndex]);
            double elapsedTime = (System.nanoTime()-start)/1000000000.0;
            totalTime += elapsedTime;

            boolean x = gamepad1.x;
            if (x && !lastX){
                servoIndex += drive.servos.size() - 1;
            }
            lastX = x;
            boolean y = gamepad1.y;
            if (y && !lastY){
                servoIndex += 1;
            }
            lastY = y;
            servoIndex = servoIndex % drive.servos.size();
            if (gamepad1.left_bumper){
                drive.duckSpin.setPower(1.0);
                drive.duckSpin2.setPower(-1.0);
            }
            else if (gamepad1.right_bumper){
                drive.duckSpin.setPower(-1.0);
                drive.duckSpin2.setPower(1.0);
            }
            else {
                drive.duckSpin.setPower(0.0);
                drive.duckSpin2.setPower(0.0);
            }


            telemetry.addData("servoNum", servoIndex);
            telemetry.addData("servoPos", servoPos[servoIndex]);
            telemetry.addData("averageServoTime", totalTime/numLoops);
            telemetry.addData("left", drive.magLeft.getVoltage());
            telemetry.addData("right", drive.magRight.getVoltage());
            telemetry.update();
        }
    }
}
