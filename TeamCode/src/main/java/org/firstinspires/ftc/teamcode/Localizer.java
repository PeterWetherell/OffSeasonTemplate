package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class Localizer {
    public Encoder[] encoders;
    long lastTime = System.nanoTime();
    double x = 0;
    double y = 0;
    double heading = 0;
    double headingOffset = 0;
    double startingHeading = 0;
    Pose2d currentPose;
    Pose2d currentVel;
    Pose2d relCurrentVel;

    ArrayList<Pose2d> poseHistory = new ArrayList<Pose2d>();
    ArrayList<Pose2d> relHistory = new ArrayList<Pose2d>();
    ArrayList<Double> loopTimes = new ArrayList<Double>();

    public Localizer(){
        encoders = new Encoder[3];
        encoders[0] = new Encoder(new Pose2d(0.125,-4.809341+0.125),1.0);
        encoders[1] = new Encoder(new Pose2d(0.125,4.7688719-0.125),  -1.0);
        encoders[2] = new Encoder(new Pose2d(6.09063296,3),  1.0);

        currentPose = new Pose2d(0,0,0);
    }
    public void updateEncoders(int[] encoders){
        for (int i = 0; i < this.encoders.length; i ++){
            this.encoders[i].update(encoders[i]);
        }
    }
    public void update(){
        long currentTime = System.nanoTime();
        double loopTime = (currentTime-lastTime)/1000000000.0;
        lastTime = currentTime;

        double deltaRight = encoders[0].getDelta();
        double deltaLeft = encoders[1].getDelta();
        double deltaBack = encoders[2].getDelta();

        //This is the heading because the heading is proportional to the difference between the left and right wheel.
        double deltaHeading = (deltaRight - deltaLeft)/Math.abs(encoders[1].y-encoders[0].y);
        double odoHeading = (encoders[0].getCurrentDist() - encoders[1].getCurrentDist())/(Math.abs(encoders[1].y-encoders[0].y));
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
