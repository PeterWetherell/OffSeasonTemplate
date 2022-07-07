package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class Trajectory {

    private ArrayList<Pose2d> internalPoints;
    boolean slowDown;
    public Trajectory (Pose2d a, boolean slowDown){
        internalPoints = new ArrayList<>();
        internalPoints.add(a);
        this.slowDown = slowDown;
    }
    public Trajectory addLine(Pose2d end){
        Pose2d start = internalPoints.get(internalPoints.size()-1);
        internalPoints.set(internalPoints.size() - 1, new Pose2d(
                        start.x,
                        start.y,
                        start.heading,
                        end.headingOffset,
                        end.radius,
                        start.speed
                ));
        double i = 0;
        ArrayList<Pose2d> newPoints = new ArrayList<>();
        double d = Math.sqrt(Math.pow(end.x-internalPoints.get(internalPoints.size()-1).x,2)+Math.pow(end.y-internalPoints.get(internalPoints.size()-1).y,2));
        while (i <= 1){
            i += 0.01;
            newPoints.add(
                    new Pose2d(
                            start.x + (end.x-start.x) * i,
                            start.y + (end.y-start.y) * i,
                            end.heading,
                            end.headingOffset,
                            end.radius,
                            Math.min(Math.max((d - end.radius - 4) / 16 * i * (end.speed - start.speed) + start.speed, Math.min(start.speed,end.speed)),Math.max(start.speed,end.speed))
                    )
            );
        }
        internalPoints.addAll(newPoints);
        Trajectory a = new Trajectory(new Pose2d(0,0), slowDown);
        a.internalPoints = internalPoints;
        return a;
    }
    public Trajectory end(){
        if (slowDown){
            internalPoints.get(internalPoints.size()-1).radius = 4;
            internalPoints.get(internalPoints.size()-1).speed = 0.20;
            for (int i = 2; i <= internalPoints.size(); i ++){
                double d = Math.sqrt(Math.pow(internalPoints.get(internalPoints.size() - i).x-internalPoints.get(internalPoints.size()-1).x,2)+Math.pow(internalPoints.get(internalPoints.size() - i).y-internalPoints.get(internalPoints.size()-1).y,2)) + internalPoints.get(internalPoints.size()-i).radius;
                double speed = 0.20 + Math.max((d-8)/15,0);
                double radius = 4 + Math.max((d-5) * 0.6666,0);
                if (speed >= 1){
                    Trajectory a = new Trajectory(new Pose2d(0,0), slowDown);
                    a.internalPoints = internalPoints;
                    return a;
                }
                internalPoints.get(internalPoints.size()-i).speed = Math.min(internalPoints.get(internalPoints.size()-i).speed,speed);
                internalPoints.get(internalPoints.size()-i).radius = Math.min(internalPoints.get(internalPoints.size()-i).radius,radius);
            }
        }
        Trajectory a = new Trajectory(new Pose2d(0,0), slowDown);
        a.internalPoints = internalPoints;
        return a;
    }
    public ArrayList<Pose2d> points;
    public ArrayList<Pose2d> pastPoints;
    public void start(){
        points = internalPoints;
        pastPoints = new ArrayList<>();
    }
    double getDist(Pose2d p1, Pose2d p2){
        return Math.sqrt(Math.pow(p1.x-p2.x,2) + Math.pow(p1.y-p2.y,2));
    }
    public void update(Pose2d currentPose, Pose2d relCurrentVel){
        double vel = Math.sqrt(Math.pow(relCurrentVel.x, 2) + Math.pow(relCurrentVel.y, 2));
        while (points.size() > 1 && Math.sqrt(Math.pow(currentPose.x - points.get(0).x, 2) + Math.pow(currentPose.y - points.get(0).y, 2)) <= points.get(0).radius) {
            pastPoints.add(points.remove(0));
        }
        while (pastPoints.size() >= 2 && getDist(currentPose,pastPoints.get(0)) > getDist(currentPose,pastPoints.get(1))){
            pastPoints.remove(0);
        }
        if (points.size() == 1){
            if (slowDown){
                double headingError = currentPose.getHeading() - points.get(0).heading;
                while (Math.abs(headingError) > Math.PI){
                    headingError -= Math.PI * 2 * Math.signum(headingError);
                }
                if (vel <= 4 && Math.sqrt(Math.pow(currentPose.x - points.get(0).x, 2) + Math.pow(currentPose.y - points.get(0).y, 2)) <= 2 && Math.abs(Math.toDegrees(headingError)) <= 5){
                    points.remove(0);
                }
            }
            else{
                if (Math.sqrt(Math.pow(currentPose.x - points.get(0).x, 2) + Math.pow(currentPose.y - points.get(0).y, 2)) <= points.get(0).radius){
                    points.remove(0);
                }
            }
        }
    }
}