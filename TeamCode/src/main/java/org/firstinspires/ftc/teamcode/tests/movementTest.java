package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Pose2d;
import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

import java.util.ArrayList;

@TeleOp
public class movementTest extends LinearOpMode {
    SampleMecanumDrive drive;
    ArrayList<Pose2d> points = new ArrayList<>();
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
        addLine(new Pose2d(-12,-12), new Pose2d(60,-12), 0, 1, 18);
        addLine(new Pose2d(60,-12), new Pose2d(60,60), 0, 1, 18);
        addLine(new Pose2d(60,60), new Pose2d(-12,60), 0, 1, 18);
        addLine(new Pose2d(-12,60), new Pose2d(-12,-12), 0, 1, 18);
        followLine();
    }
    public void followLine(){
        while (opModeIsActive() && points.size() > 1) {
            while (points.size() > 1 && Math.sqrt(Math.pow(drive.currentPose.x - points.get(0).x, 2) + Math.pow(drive.currentPose.y - points.get(0).y, 2)) <= points.get(0).radius) {
                points.remove(0);
            }
            drive.target = points.get(0);
            moveToPoint(points.get(0));
            drive.update();
        }
        while (opModeIsActive() && Math.sqrt(Math.pow(drive.currentPose.x - points.get(0).x, 2) + Math.pow(drive.currentPose.y - points.get(0).y, 2)) >= 2){
            double error = Math.sqrt(Math.pow(drive.currentPose.x - points.get(0).x, 2) + Math.pow(drive.currentPose.y - points.get(0).y, 2));
            double errorX = drive.currentPose.x - points.get(0).x;
            double errorY = drive.currentPose.y - points.get(0).y;
            double headingError = (points.get(0).heading + points.get(0).headingOffset) - drive.currentPose.heading;
            while (headingError >= Math.PI){
                headingError -= 2 * Math.PI;
            }
            while (headingError <= -Math.PI){
                headingError += 2 * Math.PI;
            }
            double relErrorX = errorX * Math.cos(drive.currentPose.heading) - errorY * Math.sin(drive.currentPose.heading);
            double relErrorY = errorY * Math.cos(drive.currentPose.heading) + errorX * Math.sin(drive.currentPose.heading);
            double v = Math.abs(relErrorX) + Math.abs(relErrorY);
            double velX = relErrorX/v * points.get(0).speed * Math.min(error/6.0,1.0);
            double velY = relErrorY/v * points.get(0).speed * Math.min(error/6.0,1.0);
            double turn = Math.toDegrees(headingError)/30.0;
            double p1 = velX-velY-turn;
            double p2 = velX+velY-turn;
            double p3 = velX-velY+turn;
            double p4 = velX+velY+turn;
            drive.pinMotorPowers(p1, p2, p3, p4);
            drive.update();
        }
        points.clear();
        drive.target = null;
        drive.setMotorPowers(0,0,0,0);
    }
    public void addLine(Pose2d start, Pose2d end, double offsetHeading, double speed, double error){
        double i = 0;
        while (i <= 1){
            points.add(new Pose2d(start.x + (end.x-start.x) * i,start.y + (end.y-start.y) * i, Math.atan2(end.y - start.y,end.x - start.x), offsetHeading, error, speed));
            i += 0.01;
        }
    }
    public void moveToPoint(Pose2d target){
        double targetAngle = Math.atan2(target.y - drive.currentPose.y,target.x - drive.currentPose.x);
        double error = Math.sqrt(Math.pow(drive.currentPose.x - target.x,2) + Math.pow(drive.currentPose.y - target.y,2));
        targetAngle += target.headingOffset;
        double errorHeading = targetAngle - drive.currentPose.getHeading();
        while (errorHeading >= Math.PI){
            errorHeading -= 2 * Math.PI;
        }
        while (errorHeading <= -Math.PI){
            errorHeading += 2 * Math.PI;
        }
        double turn = Math.toDegrees(errorHeading)/30.0;

        double relErrorX = error * Math.cos(errorHeading);
        double relErrorY = error * Math.sin(errorHeading);

        final double v = Math.abs(relErrorX) + Math.abs(relErrorY);
        double velX = target.speed * relErrorX / v;
        double velY = target.speed * relErrorY / v;

        double p1 = velX-velY-turn;
        double p2 = velX+velY-turn;
        double p3 = velX-velY+turn;
        double p4 = velX+velY+turn;
        drive.pinMotorPowers(p1, p2, p3, p4);
    }
}
