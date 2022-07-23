package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Pose2d;
import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Trajectory;
import org.firstinspires.ftc.teamcode.TrajectoryPeice;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "Auto")
public class WearhouseAutoRed extends LinearOpMode {

    SampleMecanumDrive drive;
    double side = -1;

    double lastIntakeX = 36; //39

    long start = System.currentTimeMillis();
    long cutoff = 1300;

    int numMinerals = 0;

    int numCycle = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startingPose = new Pose2d(12,65.25 * side,0);
        Pose2d endPoint = new Pose2d(12,65.125 * side,0);

        int capNum = 2;

        drive.currentIntake = side;
        drive.transferMineral = true;
        drive.setV4barOrientation(Math.toRadians(-5));
        drive.setDepositAngle(drive.depositTransferAngle);
        drive.setTurretTarget(drive.intakeTurretInterfaceHeading * drive.currentIntake);
       // drive.updateSlideLength = false;

        setUp(startingPose);
        /*
        Logger a = new Logger("Alliance",false);
        String l = "red";
        a.addData(l);
        a.update();
        a.close();
         */
        while (!isStarted() && !isStopRequested()) {
            drive.update();
        }


        setUp(startingPose);

        start = System.currentTimeMillis();
        drive.servos.get(5).setPosition(0);
        drive.servos.get(6).setPosition(1.0);
        drive.v4barOffset = Math.toRadians(-4);
        //drive.updateSlideLength = true;
        depositFirst(capNum, endPoint);
        drive.intakeLiftDelay = 0; //50
        while (((System.currentTimeMillis() - start) <= (30000 - 2000 - 500)) && opModeIsActive()){
            driveIn(endPoint);
            if (((System.currentTimeMillis() - start) <= (30000 - 2000)) && opModeIsActive()){
                driveOut(endPoint);
            }
        }
        if (drive.intakeCase <= 2) {
            drive.intakeCase = 0;
        }

        while (opModeIsActive()){

        }
        long opModeEnded = System.currentTimeMillis();
        while (System.currentTimeMillis()-opModeEnded >= 500){

        }
        //driveToPoint(new Pose2d(40, endPoint.getY(), 0),new Pose2d(72, 24, 0), false, 1, 0.85, 1000, 13, true,100);
        drive.setMotorPowers( 0.2, 0.2, 0.2, 0.2);
        drive.slides.setPower(0);
        drive.slides2.setPower(0);
        drive.turret.setPower(0);
    }
    public void driveIn(Pose2d endPoint){
        numCycle ++;
        drive.startIntake(side == -1);
        double angle = Math.toRadians(-10) * side * (numMinerals % 3);
        double x =  -2 + lastIntakeX + 12 * (1 - Math.cos(angle)); // -2, (-)
        double y = 71.25 * Math.signum(endPoint.getY()) - Math.sin(angle) * -8.0 - Math.cos(angle) * 6.0 * side;
        if (angle != 0) {
            drive.followTrajectory(this,
                    new Trajectory(new TrajectoryPeice(drive.currentPose, 0, 5, 0.5), true)
                            .addLine(new TrajectoryPeice(new Pose2d(x - 5, endPoint.y, 0), 0, 5, 0.95))
                            .addLine(new TrajectoryPeice(new Pose2d(x, y, angle), 0, 4, 0.6))
                            .end()
            );
        }
        else{
            drive.followTrajectory(this,
                    new Trajectory(new TrajectoryPeice(drive.currentPose, 0, 5, 0.5), true)
                            .addLine(new TrajectoryPeice(new Pose2d(x, endPoint.y, 0), 0, 5, 0.95))
                            .end()
            );
            int k = 6;
            if (numMinerals == 0){
                k = 2;
            }
            drive.intakeMineral(this,0.5,k * 150, false);
        }
        if (Math.abs(drive.currentPose.getHeading()) >= Math.toRadians(5) || angle == 0) { //make it so that if it intakes something while coming in that it then keeps going at the same angle the next cycle through
            numMinerals++;
        }
        drive.intakeMineral(this,0.285,2000, true);
        if (drive.intakeCase == 2){
            drive.intakeCase ++;
        }
        lastIntakeX = Math.min(54,Math.max(drive.currentPose.getX(),lastIntakeX + 2));
    }
    public void driveOut(Pose2d endPoint){
        double offset = 1.5; //1.5
        if (numCycle >= 7){
            offset = 0;
        }
        drive.effectiveDepositAngle = Math.toRadians(-35);//-35
        drive.v4barOffset = Math.toRadians(-6); drive.slidesOffset = 1.0; drive.turretOffset = Math.toRadians(0 * side); // 4 => 5 => -4, -4 => -6, 0.5
        Pose2d newEnd = new Pose2d(endPoint.getX() + offset, endPoint.getY(), endPoint.getHeading());
        drive.startDeposit(endPoint, new Pose2d(-13, 24.0 * Math.signum(endPoint.getY())),13.5,3);
        drive.followTrajectory(this,
                new Trajectory(new TrajectoryPeice(drive.currentPose, 0, 5, 0.5), true)
                        .addLine(new TrajectoryPeice(new Pose2d(36.5, newEnd.getY() + side * 0.2, 0), 0, 5, 0.85))
                        .addLine(new TrajectoryPeice(new Pose2d(newEnd.getX() + 3.5,newEnd.getY() + 0.25 * side, 0), 0, 4, 0.6))
                        .end()
        );
        drive.depositAtPoint(this, newEnd);
    }
    public void setUp(Pose2d startingPose){
        drive.update();
        drive.setPose(startingPose.x,startingPose.y,startingPose.heading);
        drive.update();
    }
    public void depositFirst(int capNum, Pose2d endPoint){
        double h = 20;
        double r = 6;
        double offset = 0;
        switch (capNum) {
            case 0: r = 9.25; h = 2.5; drive.slidesOffset = -3.25; offset = -1; drive.turretOffset = Math.toRadians(3.5 * side); break; //-6
            case 1: r = 7; h = 7.125; drive.v4barOffset = Math.toRadians(2); offset = 1.25; drive.turretOffset = Math.toRadians(3.5 * side); break; //-6.2 => 2
            case 2: r = 3; h = 13.5; break;
        }
        drive.startDeposit(endPoint, new Pose2d(-12.0, 24.0 * Math.signum(endPoint.getY())),h,r);
        waitForDepositSlow(offset);
        drive.slidesOffset = 0;
        drive.turretOffset = 0;
        drive.v4barOffset = 0;
    }
    public void waitForDepositSlow(double offset){
        while (drive.slidesCase < 3 && opModeIsActive()){
            drive.update();
            if(drive.currentIntake == 1){drive.servos.get(1).setPosition(drive.leftIntakeDrop);}
            if(drive.currentIntake == -1){drive.servos.get(0).setPosition(drive.rightIntakeDrop);}
        }
        while (drive.slidesCase <= 4 && opModeIsActive()) {
            drive.slidesOffset = offset;
            drive.deposit();
            drive.update();
            if(drive.currentIntake == 1){drive.servos.get(1).setPosition(drive.leftIntakeDrop);}
            if(drive.currentIntake == -1){drive.servos.get(0).setPosition(drive.rightIntakeDrop);}
        }
    }
}