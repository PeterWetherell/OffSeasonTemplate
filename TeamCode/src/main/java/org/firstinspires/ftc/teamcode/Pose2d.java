package org.firstinspires.ftc.teamcode;

public class Pose2d {
    public double x;
    public double y;
    public double heading;
    public double headingOffset;
    public double radius;
    public double speed;
    public Pose2d (double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
    public Pose2d (double x, double y, double heading, double headingOffset, double radius, double speed){
        this.x = x;
        this.y = y;
        this.headingOffset = headingOffset;
        this.heading = heading;
        this.radius = radius;
        this.speed = speed;
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
    public Pose2d (double x, double y){
        this(x,y,0);
    }
}
