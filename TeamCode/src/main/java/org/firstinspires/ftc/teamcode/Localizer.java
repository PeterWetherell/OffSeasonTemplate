package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class Localizer {
    public Encoder[] encoders;
    long lastTime = System.nanoTime();
    public double x = 0;
    public double y = 0;
    double heading = 0;
    Pose2d currentPose = new Pose2d(0,0,0);
    Pose2d currentVel = new Pose2d(0,0,0);
    Pose2d relCurrentVel = new Pose2d(0,0,0);
    Pose2d currentPowerVector = new Pose2d(0,0,0);

    ArrayList<Pose2d> poseHistory = new ArrayList<Pose2d>();
    ArrayList<Pose2d> relHistory = new ArrayList<Pose2d>();
    ArrayList<Double> loopTimes = new ArrayList<Double>();

    Pose2d leftSensor = new Pose2d(0,0);
    Pose2d rightSensor = new Pose2d(0,0);

    public void setPose(double x, double y, double h){
        this.x = x;
        this.y = y;
        this.heading += h - this.heading;
        lastIMUCall = System.currentTimeMillis();
        lastPose = new Pose2d(x,y,h);
        lastImuHeading = imu.getAngularOrientation().firstAngle;
    }

    BNO055IMU imu;
    public void getIMU(BNO055IMU imu){
        this.imu = imu;
    }
    public Localizer(){
        encoders = new Encoder[3];
        encoders[0] = new Encoder(new Pose2d(0.125,-4.119918272),1.013856046);
        encoders[1] = new Encoder(new Pose2d(0.125,5.314013086),  -1.013856046);
        encoders[2] = new Encoder(new Pose2d(2,0.125),  -1.002930355);//6.2232,3,1
    }
    public void updateEncoders(int[] encoders){
        for (int i = 0; i < this.encoders.length; i ++){
            this.encoders[i].update(encoders[i]);
        }
    }
    int numWallUpdate = 0;
    public void wallUpdate(double val){
        double valTouchIn = 100;
        double valTouchOut = 380;
        if (val < valTouchOut){
            double a = 6.75 + (val - valTouchIn)/(valTouchOut - valTouchIn) * 0.5;
            //5.0 represents the distance forward/back from center of robot
            //6.75 represents the base and then the equation is being used to determine its actual full extent for left/right.
            double sensorX = x + Math.cos(heading) * 5.0 - Math.sin(heading) * a * Math.signum(y);
            double sensorY = y + Math.cos(heading) * a * Math.signum(y) + Math.sin(heading) * 5.0;
            if (Math.abs(sensorX) >= 68){
                numWallUpdate = Math.min(numWallUpdate + 1, 10);
                if (numWallUpdate == 10) {
                    x += (72 * Math.signum(sensorX) - sensorX) * 0.1;
                }
            }
            else if (Math.abs(sensorY) >= 68){
                numWallUpdate = Math.min(numWallUpdate + 1, 10);
                if (numWallUpdate == 10) {
                    y += (72 * Math.signum(sensorY) - sensorY) * 0.1;
                }
            }
            else{
                numWallUpdate = 0;
            }
        }
    }

    long startSideWallRamSide = System.currentTimeMillis();
    long lastSideWallRamSide = System.currentTimeMillis();
    long startSideWallRamFront = System.currentTimeMillis();
    long lastSideWallRamFront = System.currentTimeMillis();

    public void ramWallUpdate(){
        double robotWidth = 12;
        double robotLength = 17;

        if (Math.abs(currentVel.y) <= 3 && Math.abs(currentPowerVector.y) >= 0.25){
            lastSideWallRamSide = System.currentTimeMillis();
            if (System.currentTimeMillis() - startSideWallRamSide >= 800 && Math.signum(currentPowerVector.y) == Math.signum(currentPose.y)){
                y = (72 - (Math.abs(Math.cos(heading)) * robotWidth / 2.0 + Math.abs(Math.sin(heading)) * robotLength / 2.0)) * Math.signum(currentPose.y);
            }
        }
        else{
            if (System.currentTimeMillis() - lastSideWallRamSide >= 50) {
                startSideWallRamSide = System.currentTimeMillis();
            }
        }

        if (Math.abs(currentVel.x) <= 3 && Math.abs(currentPowerVector.x) >= 0.25){
            lastSideWallRamFront = System.currentTimeMillis();
            if (System.currentTimeMillis() - startSideWallRamFront >= 800 && Math.signum(currentPowerVector.x) == Math.signum(currentPose.x)){
                x = (72 - (Math.abs(Math.cos(heading)) * robotLength / 2.0 + Math.abs(Math.sin(heading)) * robotWidth / 2.0)) * Math.signum(currentPose.x);
            }
        }
        else{
            if (System.currentTimeMillis() - lastSideWallRamFront >= 50) {
                startSideWallRamFront = System.currentTimeMillis();
            }
        }
    }
    int numDistLeft = 0;
    int numDistRight = 0;
    double wallDist = 48.0;
    public void distUpdate(double rightDist, double leftDist){
        double rightSensorX = x + Math.cos(heading) * 8.0 - Math.sin(heading) * -6.0;
        double rightSensorY = y + Math.sin(heading) * 8.0 + Math.cos(heading) * -6.0;
        double leftSensorX = x + Math.cos(heading) * 8.0 - Math.sin(heading) *  6.0;
        double leftSensorY = y + Math.sin(heading) * 8.0 + Math.cos(heading) *  6.0;
        double normHeading = Math.toDegrees(heading);
        while (Math.abs(normHeading) > 180){
            normHeading -= Math.signum(normHeading)*360;
        }
        normHeading = Math.abs(90 - Math.abs(heading));
        double xErrorLeft = 0;
        double yErrorLeft = 0;
        if (Math.abs(leftSensorY) <= wallDist - 8 && Math.abs(leftSensorX) <= wallDist - 8){ //Make sure the sensor itself is not near a wall
            leftSensorX += Math.cos(heading) * leftDist;
            leftSensorY += Math.sin(heading) * leftDist;
            if ((Math.abs(Math.abs(leftSensorX)-wallDist) <= 3 && normHeading >= 70) ^ (Math.abs(Math.abs(leftSensorY)-wallDist) <= 3 && normHeading <= 20)){ //Check to make sure that the reading lines up with a wall & only 1 wall
                if (Math.abs(Math.abs(leftSensorX)-wallDist) <= 3){ //finding out which of the two walls
                    numDistLeft = Math.min(numDistLeft + 1, 6);
                    if (numDistLeft == 6) {
                        xErrorLeft = wallDist * Math.signum(leftSensorX) - leftSensorX; //this can be thought of as making leftSensorX + xError (currentError) = 72 (because it is bouncing off the wall)
                    }
                }
                else {
                    numDistLeft = Math.min(numDistLeft + 1, 6);
                    if (numDistLeft == 6) {
                        yErrorLeft = wallDist * Math.signum(leftSensorY) - leftSensorY;
                    }
                }
            }
            else{
                numDistLeft = 0;
            }
        }
        else{
            numDistLeft = 0;
            leftSensorX += Math.cos(heading) * leftDist;
            leftSensorY += Math.sin(heading) * leftDist;
        }

        double xErrorRight = 0;
        double yErrorRight = 0;
        if (Math.abs(rightSensorY) <= wallDist - 8 && Math.abs(rightSensorX) <= wallDist - 8){
            rightSensorX += Math.cos(heading) * rightDist;
            rightSensorY += Math.sin(heading) * rightDist;
            if ((Math.abs(Math.abs(rightSensorX)-wallDist) <= 3 && normHeading >= 70) ^ (Math.abs(Math.abs(rightSensorY)-wallDist) <= 3 && normHeading <= 20)){ //Check to make sure that the reading lines up with a wall & only 1 wall
                if (Math.abs(Math.abs(rightSensorX)-wallDist) <= 3){ //finding out whih of the two walls
                    numDistRight = Math.min(numDistRight + 1, 6);
                    if (numDistRight == 6) {
                        xErrorRight = wallDist * Math.signum(rightSensorX) - rightSensorX; //this can be thought of as making leftSensorX + xError (currentError) = 72 (because it is bouncing off the wall)
                    }
                }
                else {
                    numDistRight = Math.min(numDistRight + 1, 6);
                    if (numDistRight == 6) {
                        yErrorRight = wallDist * Math.signum(rightSensorY) - rightSensorY;
                    }
                }
            }
            else{
                numDistRight = 0;
            }
        }
        else {
            numDistRight = 0;
            rightSensorX += Math.cos(heading) * rightDist;
            rightSensorY += Math.sin(heading) * rightDist;
        }

        leftSensor = new Pose2d(leftSensorX,leftSensorY);
        rightSensor = new Pose2d(rightSensorX,rightSensorY);

        x += (xErrorLeft + xErrorRight)/2.0 * 0.01; // This means that the localization updates twice as fast when both sensors are in agreement
        y += (yErrorLeft + yErrorRight)/2.0 * 0.01; //0.01 is chosen at random, but because the weighted running average is inherently stable it still works
    }
    int i = 0;
    public void update(){
        if (i == 0){
            lastImuHeading = imu.getAngularOrientation().firstAngle;
        }
        i ++;
        ramWallUpdate();

        long currentTime = System.nanoTime();
        double loopTime = (currentTime-lastTime)/1000000000.0;
        lastTime = currentTime;

        double deltaRight = encoders[0].getDelta();
        double deltaLeft = encoders[1].getDelta();
        double deltaBack = encoders[2].getDelta();
        double rightY = encoders[0].y;
        double leftY = encoders[1].y;
        double backX = encoders[2].x;

        //This is the heading because the heading is proportional to the difference between the left and right wheel.
        double deltaHeading = (deltaRight - deltaLeft)/(leftY-rightY);
        //This gives us deltaY because the back minus theta*R is the amount moved to the left minus the amount of movement in the back encoder due to change in heading
        double relDeltaY = deltaBack - deltaHeading*backX;
        //This is a weighted average for the amount moved forward with the weights being how far away the other one is from the center
        double relDeltaX = (deltaLeft*rightY - deltaRight*leftY)/(leftY-rightY);

        relHistory.add(0,new Pose2d(relDeltaX,relDeltaY,deltaHeading));


        if (deltaHeading != 0) { // this avoids the issue where deltaHeading = 0 and then it goes to undefined. This effectively does L'Hopital's
            double r1 = relDeltaX / deltaHeading;
            double r2 = relDeltaY / deltaHeading;
            relDeltaX = Math.sin(deltaHeading) * r1 - (1.0 - Math.cos(deltaHeading)) * r2;
            relDeltaY = (1.0 - Math.cos(deltaHeading)) * r1 + Math.sin(deltaHeading) * r2;
        }
        x += relDeltaX * Math.cos(heading) - relDeltaY * Math.sin(heading);
        y += relDeltaY * Math.cos(heading) + relDeltaX * Math.sin(heading);

        heading += deltaHeading;

        currentPose = new Pose2d(x, y, heading);

        loopTimes.add(0,loopTime);
        poseHistory.add(0,currentPose);
        updateVelocity();
        updateStrafeHeading();
    }
    long lastIMUCall = System.currentTimeMillis();
    Pose2d lastPose = new Pose2d(0,0,0);
    double lastImuHeading = 0;
    public void updateStrafeHeading(){
        //if (Math.abs(relCurrentVel.getX()) >= 6 || Math.abs(relCurrentVel.getX())/Math.max(Math.abs(relCurrentVel.getY()),0.1) >= 0.5){} //check strafing
        if (System.currentTimeMillis() - lastIMUCall >= 200){
            double deltaX = x-lastPose.x;
            double deltaY = y-lastPose.y;
            double imuHeading = imu.getAngularOrientation().firstAngle;
            double deltaHeading = (imuHeading - lastImuHeading) - (heading - lastPose.heading);
            while (Math.abs(deltaHeading) > Math.PI){
                deltaHeading -= Math.PI * 2 * Math.signum(deltaHeading);
            }
            x = lastPose.x + deltaX * Math.cos(deltaHeading) - deltaY * Math.sin(deltaHeading);
            y = lastPose.y + deltaX * Math.sin(deltaHeading) + deltaY * Math.cos(deltaHeading);
            heading += deltaHeading;

            lastIMUCall = System.currentTimeMillis();
            lastPose = new Pose2d(x,y,heading);
            lastImuHeading = imuHeading;
        }
    }
    public void updatePowerVector(double[] p){
        for (int i = 0; i < p.length; i ++){
            p[i] = Math.max(Math.min(p[i],1),-1);
        }
        double forward = (p[0] + p[1] + p[2] + p[3]) / 4;
        double left = (-p[0] + p[1] - p[2] + p[3]) / 4; //left power is less than 1 of forward power
        double turn = (-p[0] - p[1] + p[2] + p[3]) / 4;
        currentPowerVector.x = forward * Math.cos(heading) - left * Math.sin(heading);
        currentPowerVector.y = left * Math.cos(heading) + forward * Math.sin(heading);
        currentPowerVector.heading = turn;
    }
    public void updateVelocity(){
        double targetVelTimeEstimate = 0.2;
        double actualVelTime = 0;
        double relDeltaXTotal = 0;
        double relDeltaYTotal = 0;
        double totalTime = 0;
        int lastIndex = 0;
        for (int i = 0; i < loopTimes.size(); i ++){
            totalTime += loopTimes.get(i);
            if (totalTime <= targetVelTimeEstimate){
                actualVelTime += loopTimes.get(i);
                relDeltaXTotal += relHistory.get(i).getX();
                relDeltaYTotal += relHistory.get(i).getY();
                lastIndex = i;
            }
        }
        currentVel = new Pose2d(
                (poseHistory.get(0).getX() - poseHistory.get(lastIndex).getX()) / actualVelTime,
                (poseHistory.get(0).getY() - poseHistory.get(lastIndex).getY()) / actualVelTime,
                (poseHistory.get(0).getHeading() - poseHistory.get(lastIndex).getHeading()) / actualVelTime
        );
        relCurrentVel = new Pose2d(
                (relDeltaXTotal) / actualVelTime,
                (relDeltaYTotal) / actualVelTime,
                (poseHistory.get(0).getHeading() - poseHistory.get(lastIndex).getHeading()) / actualVelTime
        );
        while (lastIndex + 1 < loopTimes.size()){
            loopTimes.remove(loopTimes.size() - 1);
            relHistory.remove(relHistory.size() - 1);
            poseHistory.remove(poseHistory.size() - 1);
        }
    }
}
