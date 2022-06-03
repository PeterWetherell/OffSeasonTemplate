package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class Localizer {
    public Encoder[] encoders;
    long lastTime = System.nanoTime();
    public double x = 0;
    public double y = 0;
    double heading = 0;
    double odoHeading = 0;
    double headingOffset = 0;
    double startingHeading = 0;
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
        this.startingHeading += h - this.heading;
    }

    Pose2d lastUpdatePose = new Pose2d(0,0);
    boolean firstUpdate = false;
    public void updateHeading(double h){
        if(!firstUpdate){
            lastUpdatePose = new Pose2d(x,y);
        }
        double deltaX = x - lastUpdatePose.x;
        double deltaY = y - lastUpdatePose.y;
        double headingError = (h - odoHeading);
        while (headingError >= Math.PI){
            headingError -= Math.PI * 2;
        }
        while (headingError <= -Math.PI){
            headingError += Math.PI * 2;
        }
        this.headingOffset += headingError;

        x += deltaX * Math.cos(headingError) - deltaY * Math.sin(headingError);
        y += deltaY * Math.cos(headingError) + deltaX * Math.sin(headingError);

        lastUpdatePose = new Pose2d(x,y);
    }

    public Localizer(){
        encoders = new Encoder[3];
        encoders[0] = new Encoder(new Pose2d(0.125,-4.119918272),1.013856046);
        encoders[1] = new Encoder(new Pose2d(0.125,5.314013086),  -1.013856046);
        encoders[2] = new Encoder(new Pose2d(0.125,3),  -1.002930355);//6.2232,3,1

        currentPose = new Pose2d(0,0,0);
    }
    public void updateEncoders(int[] encoders){
        for (int i = 0; i < this.encoders.length; i ++){
            this.encoders[i].update(encoders[i]);
        }
    }
    public void wallUpdate(double val){
        double valTouchIn = 100;
        double valTouchOut = 380;
        if (val < valTouchOut){
            double a = 6.75 + (val - valTouchIn)/(valTouchOut - valTouchIn) * 0.5;
            //5.0 represents the distance forward/back from center of robot
            //6.75 represents the base and then the equation is being used to determine its actual full extent for left/right.
            double sensorX = Math.cos(heading) * 5.0 - Math.sin(heading) * a * Math.signum(y);
            double sensorY = Math.cos(heading) * a * Math.signum(y) + Math.sin(heading) * 5.0;
            if (Math.abs(x + sensorX) >= 68){
                //x = 72 * Math.signum(sensorX + x) - sensorX;
            }
            if (Math.abs(y + sensorY) >= 68){
                //y = 72 * Math.signum(sensorY + y) - sensorY;
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
    public void distUpdate(double rightDist, double leftDist){
        double rightSensorX = x + Math.cos(heading) * 8.0 - Math.sin(heading) * -6.0;
        double rightSensorY = y + Math.sin(heading) * 8.0 + Math.cos(heading) * -6.0;
        double leftSensorX = x + Math.cos(heading) * 8.0 - Math.sin(heading) *  6.0;
        double leftSensorY = y + Math.sin(heading) * 8.0 + Math.cos(heading) *  6.0;

        double xErrorLeft = 0;
        double yErrorLeft = 0;
        if (Math.abs(leftSensorY) <= 64 && Math.abs(leftSensorX) <= 64){ //Make sure the sensor itself is not near a wall
            leftSensorX += Math.cos(heading) * leftDist;
            leftSensorY += Math.sin(heading) * leftDist;
            if (Math.abs(leftSensorX) >= 72 - 6 ^ Math.abs(leftSensorY) >= 72 - 6){ //Check to make sure that the reading lines up with a wall & only 1 wall
                if (Math.abs(leftSensorX) >= 72 - 6){ //finding out which of the two walls
                    xErrorLeft = 72 * Math.signum(leftSensorX) - leftSensorX; //this can be thought of as making leftSensorX + xError (currentError) = 72 (because it is bouncing off the wall)
                }
                else {
                    yErrorLeft = 72 * Math.signum(leftSensorY) - leftSensorY;
                }
            }
        }
        else{
            leftSensorX += Math.cos(heading) * leftDist;
            leftSensorY += Math.sin(heading) * leftDist;
        }

        double xErrorRight = 0;
        double yErrorRight = 0;
        if (Math.abs(rightSensorY) <= 64 && Math.abs(rightSensorX) <= 64){
            rightSensorX += Math.cos(heading) * rightDist;
            rightSensorY += Math.sin(heading) * rightDist;
            if (Math.abs(rightSensorX) >= 72 - 6 ^ Math.abs(rightSensorY) >= 72 - 6){ //Check to make sure that the reading lines up with a wall & only 1 wall
                if (Math.abs(rightSensorX) >= 72 - 6){ //finding out which of the two walls
                    xErrorRight = 72 * Math.signum(rightSensorX) - rightSensorX; //this can be thought of as making leftSensorX + xError (currentError) = 72 (because it is bouncing off the wall)
                }
                else {
                    yErrorRight = 72 * Math.signum(rightSensorY) - rightSensorY;
                }
            }
        }
        else {
            rightSensorX += Math.cos(heading) * rightDist;
            rightSensorY += Math.sin(heading) * rightDist;
        }

        leftSensor = new Pose2d(leftSensorX,leftSensorY);
        rightSensor = new Pose2d(rightSensorX,rightSensorY);

        //x += (xErrorLeft + xErrorRight)/2.0 * 0.01; // This means that the localization updates twice as fast when both sensors are in agreement
        //y += (yErrorLeft + yErrorRight)/2.0 * 0.01; //0.01 is chosen at random, but because the weighted running average is inherently stable it still works
    }
    public void update(){
        ramWallUpdate();

        long currentTime = System.nanoTime();
        double loopTime = (currentTime-lastTime)/1000000000.0;
        lastTime = currentTime;

        double deltaRight = encoders[0].getDelta();
        double deltaLeft = encoders[1].getDelta();
        double deltaBack = encoders[2].getDelta();

        //This is the heading because the heading is proportional to the difference between the left and right wheel.
        double deltaHeading = (deltaRight - deltaLeft)/Math.abs(encoders[1].y-encoders[0].y);
        odoHeading = (encoders[0].getCurrentDist() - encoders[1].getCurrentDist())/(Math.abs(encoders[1].y-encoders[0].y));
        heading = odoHeading + headingOffset + startingHeading;
        //This gives us deltaY because the back minus theta*R is the amount moved to the left minus the amount of movement in the back encoder due to change in heading
        double relDeltaY = deltaBack - deltaHeading*encoders[2].x;
        //This is a weighted average for the amount moved forward with the weights being how far away the other one is from the center
        double relDeltaX = (deltaLeft*encoders[0].y - deltaRight*encoders[1].y)/(encoders[0].y-encoders[1].y);

        double[] delta = getDeltas(relDeltaX, relDeltaY, deltaHeading, heading);
        x += delta[0];
        y += delta[1];
        currentPose = new Pose2d(x, y, heading);

        loopTimes.add(0,loopTime);
        relHistory.add(0,new Pose2d(relDeltaX,relDeltaY,deltaHeading));
        poseHistory.add(0,currentPose);
        updateVelocity();
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
                (poseHistory.get(lastIndex).getX() - poseHistory.get(0).getX()) / actualVelTime,
                (poseHistory.get(lastIndex).getY() - poseHistory.get(0).getY()) / actualVelTime,
                (poseHistory.get(lastIndex).getHeading() - poseHistory.get(0).getHeading()) / actualVelTime
        );
        relCurrentVel = new Pose2d(
                (relDeltaXTotal) / actualVelTime,
                (relDeltaYTotal) / actualVelTime,
                (poseHistory.get(lastIndex).getHeading() - poseHistory.get(0).getHeading()) / actualVelTime
        );
        while (lastIndex + 1 < loopTimes.size()){
            loopTimes.remove(loopTimes.size() - 1);
            relHistory.remove(relHistory.size() - 1);
            poseHistory.remove(poseHistory.size() - 1);
        }
    }
    public double[] getDeltas(double relDeltaX, double relDeltaY, double deltaHeading, double heading){
        if (deltaHeading != 0) { // this avoids the issue where deltaHeading = 0 and then it goes to undefined. This effectively does L'Hopital's
            double r1 = relDeltaX / deltaHeading;
            double r2 = relDeltaY / deltaHeading;
            relDeltaX = Math.sin(deltaHeading) * r1 + (1.0-Math.cos(deltaHeading)) * r2;
            relDeltaY = (1.0 - Math.cos(deltaHeading)) * r1 + Math.sin(deltaHeading) * r2;
        }
        double lastHeading = heading-deltaHeading;
        double[] delta = new double[2];
        delta[0] = relDeltaX * Math.cos(lastHeading) - relDeltaY * Math.sin(lastHeading);
        delta[1] = relDeltaY * Math.cos(lastHeading) + relDeltaX * Math.sin(lastHeading);
        return delta;
    }
}
