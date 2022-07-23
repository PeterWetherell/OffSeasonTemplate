package org.firstinspires.ftc.teamcode;

public class TrajectoryPeice extends Pose2d {
    public double headingOffset;
    public double radius;
    public double speed;
    public TrajectoryPeice(Pose2d point, double headingOffset, double radius, double speed){
        super(point.x,point.y,point.heading);
        this.headingOffset = headingOffset;
        this.radius = radius;
        this.speed = speed;
    }
    public TrajectoryPeice(double x, double y, double heading, double headingOffset, double radius, double speed){
        this(new Pose2d(x,y,heading),headingOffset,radius,speed);
    }
}
