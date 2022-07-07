package org.firstinspires.ftc.teamcode;

public class PID {
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public PID(double P, double I, double D){
        p=P;
        i=I;
        d=D;
    }
    double integral = 0;
    long lastLoopTime = System.nanoTime();
    double lastError = 0;
    int counter = 0;
    public void resetIntegral(){
        integral = 0;
    }
    public double update(double error){
        if (counter == 0){
            lastLoopTime = System.nanoTime() - 10000000;
        }
        long currentTime = System.nanoTime();
        double loopTime = (currentTime - lastLoopTime)/1000000000.0;
        lastLoopTime = currentTime;
        integral += error * i * loopTime;
        double derivative = d * (error - lastError)/loopTime;
        lastError = error;
        double proportion = p * error;
        counter ++;
        return proportion + integral + derivative;
    }
}
