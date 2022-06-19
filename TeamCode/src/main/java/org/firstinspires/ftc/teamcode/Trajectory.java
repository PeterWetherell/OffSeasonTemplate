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
        Pose2d start = points.get(points.size()-1);
        points.set(points.size() - 1, new Pose2d(
                        start.x,
                        start.y,
                        start.heading,
                        end.headingOffset,
                        end.radius,
                        start.speed
                ));
        double i = 0;
        ArrayList<Pose2d> newPoints = new ArrayList<>();
        double d = Math.sqrt(Math.pow(end.x-points.get(points.size()-1).x,2)+Math.pow(end.y-points.get(points.size()-1).y,2));
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
        points.addAll(newPoints);
        Trajectory a = new Trajectory(new Pose2d(0,0), slowDown);
        a.points = points;
        return a;
    }
    public Trajectory end(){
        if (slowDown){
            points.get(points.size()-1).radius = 4;
            points.get(points.size()-1).speed = 0.20;
            for (int i = 2; i <= points.size(); i ++){
                double d = Math.sqrt(Math.pow(points.get(points.size() - i).x-points.get(points.size()-1).x,2)+Math.pow(points.get(points.size() - i).y-points.get(points.size()-1).y,2)) + points.get(points.size()-i).radius;
                double speed = 0.20 + Math.max((d-8)/15,0);
                double radius = 4 + Math.max((d-5) * 0.6666,0);
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

    public void update(Pose2d currentPose, Pose2d relCurrentVel){
        double vel = Math.sqrt(Math.pow(relCurrentVel.x, 2) + Math.pow(relCurrentVel.y, 2));
        while (points.size() > 1 && Math.sqrt(Math.pow(currentPose.x - points.get(0).x, 2) + Math.pow(currentPose.y - points.get(0).y, 2)) <= points.get(0).radius) {
            points.remove(0);
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