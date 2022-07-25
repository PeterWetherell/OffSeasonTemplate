package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class ExtraServo {
    Servo servo;
    public double speed;
    public ExtraServo(Servo servo,String servoType, double loadMultiplier){
        this.servo = servo;
        switch (servoType) {//Take the no-load speed at 4.8 V and adjust as needed based on load on servo
            case "Torque": speed = Math.toRadians(60) / 0.25; break;
            case "Speed": speed = Math.toRadians(60) / 0.11; break;
            case "Super Speed": speed = Math.toRadians(60) / 0.055; break;
        }
        speed *= loadMultiplier;
    }
    double currentOrientation = 0;
    double currentPosition = 0;
    double lastPosition = 0;
    double offset = 0;
    long lastUpdateTime = System.nanoTime();
    public double ticksPerRadians = 0.19098593171; //300* operating range so 1/(300*pi/180)
    public double intercept = 0;
    double max = 1, min = 0;
    public void servoPositions(double ticksPerRadians, double intercept, double min, double max){
        this.ticksPerRadians = ticksPerRadians;
        this.intercept = intercept;
        this.min = min;
        this.max = max;
    }
    public void setPosition(double p, double power) {
        double position = lastPosition;
        lastPosition = Math.max(Math.min(p + offset * ticksPerRadians,min),max);
        double targetOrientation = (position-intercept) / ticksPerRadians; //This is the target servo angle in radians
        long currentTime = System.nanoTime();
        double time = (double)(currentTime - lastUpdateTime)/1.0E9;
        lastUpdateTime = currentTime;
        double error = targetOrientation - currentOrientation;
        currentOrientation += Math.signum(error) * speed * power * time;
        if (Math.signum(error) != Math.signum(position - currentOrientation)){
            currentOrientation = targetOrientation;
        }
        currentPosition = currentOrientation * ticksPerRadians + intercept; //This is the current servo position in servo position
        if (power == 1) {
            servo.setPosition(position);
        }
        else{
            servo.setPosition(currentPosition);
        }
    }
    public void setOrientation(double orientation, double power){
        setPosition(orientation * ticksPerRadians + intercept,power);
    }
    public void setPosition(double orientation){
        setPosition(orientation * ticksPerRadians + intercept,1.0);
    }
    public void setOrientation(double orientation){
        setPosition(orientation * ticksPerRadians + intercept,1.0);
    }
    public double getOrientation() {
        return currentOrientation;
    }
}
