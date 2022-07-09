package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Component;
import org.firstinspires.ftc.teamcode.Encoder;
import org.firstinspires.ftc.teamcode.Point;
import org.firstinspires.ftc.teamcode.Pose2d;
import org.firstinspires.ftc.teamcode.robotComponents;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.List;

@Config
public class EncoderLocalizationTester extends LinearOpMode {
    ExpansionHubEx expansionHub1;
    public ExpansionHubMotor leftFront, leftBack, rightBack, rightFront;
    public BNO055IMU imu;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        leftFront = (ExpansionHubMotor) hardwareMap.dcMotor.get("lf");
        leftBack = (ExpansionHubMotor) hardwareMap.dcMotor.get("lr");
        rightBack = (ExpansionHubMotor) hardwareMap.dcMotor.get("rr");
        rightFront = (ExpansionHubMotor) hardwareMap.dcMotor.get("rf");

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);


        Encoder[] encoders = new Encoder[4];
        encoders[0] = new Encoder(new Pose2d(0.125,-4.119918272),1.013856046); //right
        encoders[1] = new Encoder(new Pose2d(0.125,5.314013086),  -1.013856046); //left
        encoders[2] = new Encoder(new Pose2d(2,0.125),  -1.002930355); //front
        encoders[3] = new Encoder(new Pose2d(-2,0.125),  -1.002930355); //back

        Pose2d[] poseBlock = new Pose2d[4];
        for (int i = 0; i < 4; i ++){
            poseBlock[i] = new Pose2d(0,0,0);
        }

        double lastHeading = 0;

        dashboard.setTelemetryTransmissionInterval(25);

        waitForStart();
        int loops = 0;
        double totalBulkDataTime = 0;
        double totalImuTime = 0;
        while(opModeIsActive()){
            loops ++;
            TelemetryPacket packet = new TelemetryPacket();
            Long start = System.nanoTime();
            RevBulkData bulkData = expansionHub1.getBulkInputData();
            encoders[0].update(bulkData.getMotorCurrentPosition(rightFront));
            encoders[1].update(bulkData.getMotorCurrentPosition(leftFront));
            encoders[2].update(bulkData.getMotorCurrentPosition(rightBack));
            encoders[3].update(bulkData.getMotorCurrentPosition(leftBack));
            double bulkDataTime = (System.nanoTime()-start)/1000000000.0;
            totalBulkDataTime += bulkDataTime;
            start = System.nanoTime();
            double heading = imu.getAngularOrientation().firstAngle;
            double imuTime = (System.nanoTime()-start)/1000000000.0;
            totalImuTime += imuTime;
            packet.put("BulkData Read Time",bulkDataTime);
            packet.put("Average BulkData Read Time",totalBulkDataTime/loops);
            packet.put("IMU Read Time",imuTime);
            packet.put("Average IMU Read Time",totalImuTime/loops);
            packet.put("Loops",loops);

            Canvas fieldOverlay = packet.fieldOverlay();
            for (int i = 0; i < 4; i ++){
                int a = i % 2;
                int b = 2 + i/2;
                poseBlock[i] = localizer(poseBlock[i],encoders[a],encoders[b],heading,lastHeading);
            }
            lastHeading = heading;

            drawRobot(fieldOverlay,new robotComponents(true, "#0275D8"),poseBlock[0]);
            drawRobot(fieldOverlay,new robotComponents(true, "#5CB85C"),poseBlock[1]);
            drawRobot(fieldOverlay,new robotComponents(true, "#F0AD4E"),poseBlock[2]);
            drawRobot(fieldOverlay,new robotComponents(true, "#D9534F"),poseBlock[3]);

            dashboard.sendTelemetryPacket(packet);
        }
    }
    private Pose2d localizer(Pose2d currentLocation, Encoder forward, Encoder left, double heading, double lastHeading){

        double deltaForward = forward.getDelta();
        double deltaLeft = left.getDelta();

        //This is the heading because the heading is proportional to the difference between the left and right wheel.
        double deltaHeading = (heading - lastHeading);
        while (Math.abs(deltaHeading) > Math.toRadians(180)){
            deltaHeading -= Math.toRadians(180) * Math.signum(deltaHeading);
        }
        //This gives us deltaY because the back minus theta*R is the amount moved to the left minus the amount of movement in the back encoder due to change in heading
        double relDeltaY = deltaLeft - deltaHeading*left.x;
        //This is a weighted average for the amount moved forward with the weights being how far away the other one is from the center
        double relDeltaX = deltaForward + deltaHeading*forward.y;


        if (deltaHeading != 0) { // this avoids the issue where deltaHeading = 0 and then it goes to undefined. This effectively does L'Hopital's
            double r1 = relDeltaX / deltaHeading;
            double r2 = relDeltaY / deltaHeading;
            relDeltaX = Math.sin(deltaHeading) * r1 + (1.0 - Math.cos(deltaHeading)) * r2;
            relDeltaY = (1.0 - Math.cos(deltaHeading)) * r1 + Math.sin(deltaHeading) * r2;
        }

        currentLocation.x += relDeltaX * Math.cos(lastHeading) - relDeltaY * Math.sin(lastHeading);
        currentLocation.y += relDeltaY * Math.cos(lastHeading) + relDeltaX * Math.sin(lastHeading);

        return currentLocation;
    }

    public void setDriveMode(DcMotor.RunMode runMode){
        leftFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
        rightFront.setMode(runMode);
    }
    public void drawRobot(Canvas fieldOverlay, robotComponents r, Pose2d poseEstimate){
        for (Component c : r.components){
            fieldOverlay.setStrokeWidth(c.lineRadius);
            fieldOverlay.setStroke(c.color);
            if (c.p.size() == 1){
                drawPoint(fieldOverlay,c.p.get(0),c.radius,poseEstimate);
            }
            else {
                for (int i = 1; i < c.p.size() + 1; i++) {
                    drawPoint(fieldOverlay, c.p.get(i % c.p.size()), c.radius, poseEstimate);
                    drawLine(fieldOverlay, c.p.get(i % c.p.size()), c.p.get((i - 1)%c.p.size()), poseEstimate);
                }
            }
        }
    }
    public void drawLine(Canvas c, Point p, Point p2, Pose2d pose){
        double x1 = Math.cos(pose.getHeading())*(p.x+Math.signum(p.x-p2.x)*0.5) - Math.sin(pose.getHeading())*(p.y+Math.signum(p.y-p2.y)*0.5) + pose.getX();
        double y1 = Math.cos(pose.getHeading())*(p.y+Math.signum(p.y-p2.y)*0.5) + Math.sin(pose.getHeading())*(p.x+Math.signum(p.x-p2.x)*0.5) + pose.getY();
        double x2 = Math.cos(pose.getHeading())*(p2.x+Math.signum(p2.x-p.x)*0.5) - Math.sin(pose.getHeading())*(p2.y+Math.signum(p2.y-p.y)*0.5) + pose.getX();
        double y2 = Math.cos(pose.getHeading())*(p2.y+Math.signum(p2.y-p.y)*0.5) + Math.sin(pose.getHeading())*(p2.x+Math.signum(p2.x-p.x)*0.5) + pose.getY();
        c.strokeLine(x1,y1,x2,y2);
    }
    public void drawPoint(Canvas c, Point p, double radius, Pose2d pose){
        if (radius != 0){
            double x = Math.cos(pose.getHeading())*p.x - Math.sin(pose.getHeading())*p.y + pose.getX();
            double y = Math.cos(pose.getHeading())*p.y + Math.sin(pose.getHeading())*p.x + pose.getY();
            c.strokeCircle(x,y,radius);
        }
    }
}
