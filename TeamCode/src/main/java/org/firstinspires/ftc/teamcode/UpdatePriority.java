package org.firstinspires.ftc.teamcode;

import org.openftc.revextensions2.ExpansionHubMotor;

public class UpdatePriority {
    double basePriority;
    double priorityScale;
    double lastPower = 0;
    public double power = 0;
    long lastUpdateTime;
    ExpansionHubMotor[] motor;
    public UpdatePriority(ExpansionHubMotor a, double basePriority, double priorityScale){
        this.basePriority = basePriority;
        this.priorityScale = priorityScale;
        lastUpdateTime = System.currentTimeMillis();
        motor = new ExpansionHubMotor[]{a};
    }
    public UpdatePriority(ExpansionHubMotor[] a, double basePriority, double priorityScale){
        this.basePriority = basePriority;
        this.priorityScale = priorityScale;
        lastUpdateTime = System.currentTimeMillis();
        motor = a;
    }
    public void setTargetPower(double targetPower){
        power = targetPower;
    }
    public double getPriority(double timeRemaining){
        if (power-lastPower == 0){
            lastUpdateTime = System.currentTimeMillis();
            return 0;
        }
        if (timeRemaining * 1000.0 >= 1.6 * (motor.length - 1) + 0.8){// potentially is 1.6 ms but we use this formula to allow it to slightly overshoot/undershoot
            return 0;
        }
        return basePriority + Math.abs(power-lastPower) * (System.currentTimeMillis() - lastUpdateTime) * priorityScale;
    }
    public void update(){
        for (int i = 0; i < motor.length; i ++) {
            motor[i].setPower(power);
        }
        lastUpdateTime = System.currentTimeMillis();
        lastPower = power;
    }
}
