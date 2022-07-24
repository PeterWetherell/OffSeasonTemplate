package org.firstinspires.ftc.teamcode;

public class Pose2d {
    public double x;
    public double y;
    public double heading;
    public Pose2d (double x, double y){
        this(x,y,0);
    }
    public Pose2d (double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
    public Pose2d (Pose2d pose2d, double headingOffset, double radius, double speed){
        this.x = pose2d.x;
        this.y = pose2d.y;
        this.heading = pose2d.heading;

    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getHeading(){
        return heading;
    }
}
