package org.firstinspires.ftc.teamcode;

public class TrajectoryPeice extends Pose2d {
    public double headingOffset;
    public double speed;
    public boolean intakeInterupt;
    public TrajectoryPeice(Pose2d point, double headingOffset, double speed,boolean intakeInterupt){
        super(point.x,point.y,point.heading);
        this.headingOffset = headingOffset;
        this.speed = speed;
        this.intakeInterupt = intakeInterupt;
    }
    public TrajectoryPeice(double x, double y, double heading, double headingOffset, double speed){
        this(new Pose2d(x,y,heading),headingOffset,speed,false);
    }
    public TrajectoryPeice(double x, double y, double heading, double headingOffset, double speed, boolean intake){
        this(new Pose2d(x,y,heading),headingOffset,speed,intake);
    }
    public TrajectoryPeice(Pose2d point, double headingOffset, double speed){
        this(point,headingOffset,speed,false);
    }
}
