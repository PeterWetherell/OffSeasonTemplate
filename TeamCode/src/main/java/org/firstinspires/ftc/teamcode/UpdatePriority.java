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
    public double getPriority(double timeRemaining) { // timeRemaining is in secs
        if (power-lastPower == 0){
            lastUpdateTime = System.currentTimeMillis();
            return 0;
        }
        // checks if the time remaining to be within targetLoopLength is less than 0.8 (for all motors except slides, since it is two motors).
        // if it is less than 0.8 then don't update, else update
        // Motors take 1.6ms to update so actual loopLength might be targetLoopLength + 0.8
        if (timeRemaining * 1000.0 <= 1.6 * (motor.length - 1) + 0.8) {// potentially is 1.6 ms but we use this formula to allow it to slightly overshoot/undershoot
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
