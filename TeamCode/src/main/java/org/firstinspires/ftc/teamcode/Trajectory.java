package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class Trajectory {

    public ArrayList<Pose2d> points;
    boolean slowDown;
    public Trajectory (Pose2d a, boolean slowDown){
        points = new ArrayList<>();
        points.add(a);
        this.slowDown = slowDown;
    }
    public Trajectory addLine(Pose2d end){
        points.set(points.size() - 1, new Pose2d(
                        points.get(points.size()-1).x,
                        points.get(points.size()-1).y,
                        Math.atan2(end.y - points.get(points.size()-1).y,end.x - points.get(points.size()-1).x),
                        end.headingOffset,
                        end.radius,
                        Math.max(points.get(points.size()-1).speed,0.3)
                ));
        double i = 0;
        ArrayList<Pose2d> newPoints = new ArrayList<>();
        double d = Math.sqrt(Math.pow(end.x-points.get(points.size()-1).x,2)+Math.pow(end.y-points.get(points.size()-1).y,2));
        while (i <= 1){
            newPoints.add(
                    new Pose2d(
                            points.get(points.size()-1).x + (end.x-points.get(points.size()-1).x) * i,
                            points.get(points.size()-1).y + (end.y-points.get(points.size()-1).y) * i,
                            Math.atan2(end.y - points.get(points.size()-1).y,end.x - points.get(points.size()-1).x),
                            end.headingOffset,
                            end.radius,
                            Math.max(Math.max(Math.min((d-4) / 12 * i,1),0) * (end.speed - points.get(points.size()-1).speed) + points.get(points.size()-1).speed, 0.2)
                    )
            );
            i += 0.01;
        }
        points.addAll(newPoints);
        Trajectory a = new Trajectory(new Pose2d(0,0), slowDown);
        a.points = points;
        return a;
    }
    public Trajectory end(){
        points.get(points.size()-1).radius = 4;
        if (slowDown){
            points.get(points.size()-1).speed = 0.20;
            for (int i = 2; i <= points.size(); i ++){
                double d = Math.sqrt(Math.pow(points.get(points.size() - i).x-points.get(points.size()-1).x,2)+Math.pow(points.get(points.size() - i).y-points.get(points.size()-1).y,2)) + points.get(points.size()-i).radius;
                double speed = 0.10 + Math.max((d-8)/15,0);
                double radius = 6 + Math.max((d-5)/12,0) * 8;
                if (speed >= 1){
                    Trajectory a = new Trajectory(new Pose2d(0,0), slowDown);
                    a.points = points;
                    return a;
                }
                points.get(points.size()-i).speed = Math.min(points.get(points.size()-i).speed,speed);
                points.get(points.size()-i).radius = Math.min(points.get(points.size()-i).radius,radius);
            }
        }
        Trajectory a = new Trajectory(new Pose2d(0,0), slowDown);
        a.points = points;
        return a;
    }
    public void endTraj(){
        points.clear();
    }
    public double getError(Pose2d currentPose){
        return Math.sqrt(Math.pow(currentPose.x - points.get(0).x,2) + Math.pow(currentPose.y - points.get(0).y,2));
    }

    double lastError = 100;
    double lastTurnError = 100;
    boolean errorIncreasing = false;
    double velI = 0;

    public double[] update(Pose2d currentPose, Pose2d relCurrentVel){
        double vel = Math.sqrt(Math.pow(relCurrentVel.x, 2) + Math.pow(relCurrentVel.y, 2));
        while (points.size() > 1 && Math.sqrt(Math.pow(currentPose.x - points.get(0).x, 2) + Math.pow(currentPose.y - points.get(0).y, 2)) <= points.get(0).radius) {
            points.remove(0);
        }
        if (points.size() == 1){
            if (slowDown){
                if (vel <= 4 && Math.sqrt(Math.pow(currentPose.x - points.get(0).x, 2) + Math.pow(currentPose.y - points.get(0).y, 2)) <= 2){
                    points.remove(0);
                }
            }
            else{
                if (Math.sqrt(Math.pow(currentPose.x - points.get(0).x, 2) + Math.pow(currentPose.y - points.get(0).y, 2)) <= points.get(0).radius){
                    points.remove(0);
                }
            }
        }

        if(points.size() == 0){
            return new double[]{0,0,0,0};
        }
        double targetAngle = Math.atan2(points.get(0).y - currentPose.y,points.get(0).x - currentPose.x);
        double error = Math.sqrt(Math.pow(currentPose.x - points.get(0).x,2) + Math.pow(currentPose.y - points.get(0).y,2));
        double errorHeading = targetAngle - currentPose.getHeading();
        while (errorHeading >= Math.PI){
            errorHeading -= 2 * Math.PI;
        }
        while (errorHeading <= -Math.PI){
            errorHeading += 2 * Math.PI;
        }

        double relErrorX = error * Math.cos(errorHeading);
        double relErrorY = error * Math.sin(errorHeading) * 1.55;

        double turn = errorHeading + points.get(0).headingOffset;

        if (points.size() == 1) {
            if (error > lastError || error < 3 || Math.abs(errorHeading) > Math.abs(lastTurnError) || Math.toDegrees(Math.abs(errorHeading)) <= 15) {
                errorIncreasing = true;
            }
            lastError = error;
            lastTurnError = errorHeading;
            if (errorIncreasing) {
                turn = (points.get(0).heading - currentPose.getHeading()) + points.get(0).headingOffset;
            }

            double velError = points.get(0).speed - vel/54.0;
            velI += velError * 0.008 * 0.2;
            velI = Math.min(Math.max(velI,0),1);

        }
        while (turn >= Math.PI){
            turn -= 2 * Math.PI;
        }
        while (turn <= -Math.PI){
            turn += 2 * Math.PI;
        }
        double h = Math.max(Math.abs(Math.toDegrees(turn)/55.0), 0.25) * Math.signum(turn);
        if (Math.abs(turn) <= Math.toRadians(5)){
            h = 0;
        }

        double a = velI + points.get(0).speed;

        final double v = Math.abs(relErrorX) + Math.abs(relErrorY);
        double velX = (a * relErrorX / v) * (1 - h);
        double velY = (a * relErrorY / v) * (1 - h);

        return new double[]{velX-velY-h,velX+velY-h,velX-velY+h,velX+velY+h};
    }
}
