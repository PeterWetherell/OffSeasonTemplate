package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SampleMecanumDrive {
    RevBulkData bulkData;

    ExpansionHubEx expansionHub1, expansionHub2;
    public List<ExpansionHubMotor> motors;
    public ExpansionHubMotor leftFront, leftBack, rightBack, rightFront, intake, turret, slides, slides2;

    public ArrayList<Servo> servos;
    public CRServo duckSpin, duckSpin2;

    public AnalogInput rightIntake, leftIntake, depositSensor, distLeft, distRight, magLeft, magRight, flex;
    public VoltageSensor batteryVoltageSensor;
    public ColorSensor leftWall, rightWall;
    public BNO055IMU imu;

    int[] encoders;

    double currentSlideLength = 0;
    double currentSlideSpeed = 0;
    double currentTurretAngle = 0;
    double currentIntakeSpeed = 0;

    int rightIntakeVal = 0;
    int leftIntakeVal = 0;
    int depositVal = 0;
    int flexSensorVal = 0;
    double distValLeft = 0;
    double distValRight = 0;
    int magValLeft = 0;
    int magValRight = 0;

    boolean updateHub2 = false;

    private long loopStart = System.nanoTime();
    double loopTime = 0.0;

    public ArrayList<UpdatePriority> motorPriorities;

    Localizer localizer;

    public void initMotors(HardwareMap hardwareMap){
        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        leftFront = (ExpansionHubMotor) hardwareMap.dcMotor.get("lf");
        leftBack = (ExpansionHubMotor) hardwareMap.dcMotor.get("lr");
        rightBack = (ExpansionHubMotor) hardwareMap.dcMotor.get("rr");
        rightFront = (ExpansionHubMotor) hardwareMap.dcMotor.get("rf");

        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        intake = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake");
        turret = (ExpansionHubMotor) hardwareMap.dcMotor.get("turret");
        slides = (ExpansionHubMotor) hardwareMap.dcMotor.get("slides");
        slides2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("slides2");

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        for (int i = 0; i < 4; i ++) {
            motorPriorities.add(new UpdatePriority(3,5));
        }
        motorPriorities.add(new UpdatePriority(1,2));
        motorPriorities.add(new UpdatePriority(1,3));
        motorPriorities.add(new UpdatePriority(2,6));

        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront, intake, turret, slides, slides2);
    }
    public void setDriveMode(DcMotor.RunMode runMode){
        leftFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
        rightFront.setMode(runMode);
    }
    public void initServos(HardwareMap hardwareMap){
        servos = new ArrayList<>();
        for (int i = 0; i < 12; i ++) {
            switch (i){
                case 0: servos.add(hardwareMap.servo.get("rightIntake")); break;
                case 1: servos.add(hardwareMap.servo.get("leftIntake")); break;
                case 2: servos.add(hardwareMap.servo.get("deposit")); break;
                case 3: servos.add(hardwareMap.servo.get("odoLift")); break;
                case 4: servos.add(hardwareMap.servo.get("v4bar")); break;
                case 5: servos.add(hardwareMap.servo.get("rightCapstone")); break;
                case 6: servos.add(hardwareMap.servo.get("leftCapstone")); break;
                case 7: servos.add(hardwareMap.servo.get("duckSpinSpin")); break;
                case 8: servos.add(hardwareMap.servo.get("rightOdo")); break;
                case 9: servos.add(hardwareMap.servo.get("leftOdo")); break;
            }
        }
        duckSpin = hardwareMap.crservo.get("duckSpin");
        duckSpin2 = hardwareMap.crservo.get("duckSpin2");
    }
    private void initSensors(HardwareMap hardwareMap){
        rightIntake = hardwareMap.analogInput.get("rightIntake");
        leftIntake = hardwareMap.analogInput.get("leftIntake");
        depositSensor = hardwareMap.analogInput.get("depositSensor");
        distLeft = hardwareMap.analogInput.get("distLeft");
        distRight = hardwareMap.analogInput.get("distRight");
        magLeft = hardwareMap.analogInput.get("magLeft");
        magRight = hardwareMap.analogInput.get("magRight");
        flex = hardwareMap.analogInput.get("flex");

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftWall = hardwareMap.colorSensor.get("leftWall");
        rightWall = hardwareMap.colorSensor.get("rightWall");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }
    public SampleMecanumDrive(HardwareMap hardwareMap){
        encoders = new int[3];
        initServos(hardwareMap);
        initMotors(hardwareMap);
        initSensors(hardwareMap);
        localizer = new Localizer();
    }
    public void pinMotorPowers (double v, double v1, double v2, double v3) {
        motorPriorities.get(0).setTargetPower(v);
        motorPriorities.get(1).setTargetPower(v1);
        motorPriorities.get(2).setTargetPower(v2);
        motorPriorities.get(3).setTargetPower(v3);
    }
    public void update(){
        getEncoders(); //This is the one thing that is guaranteed to occur every loop because we need encoders for odo

        loopTime = (System.nanoTime() - loopStart) / 1000000000.0; //gets the current time since the loop began
        double targetLoopLength = 0.01; //Sets the target loop time in seconds
        double a = 1;
        while(loopTime <= targetLoopLength && a >= 0){ // updates the motors while still time remaining in the loop
            int bestIndex = 0;
            double bestScore = motorPriorities.get(0).getPriority();
            for (int i = 1; i < motorPriorities.size(); i ++){ //finding the motor that is most in need of being updated;
                if (motorPriorities.get(i).getPriority() > bestScore){
                    bestIndex = i;
                    bestScore = motorPriorities.get(i).getPriority();
                }
            }
            motors.get(bestIndex).setPower(motorPriorities.get(bestIndex).power); //setting the motor of the one that most needs it
            if (bestIndex == motorPriorities.size()-1){
                slides2.setPower(motorPriorities.get(bestIndex).power); //This deals with the case of the linked mechanism for the slides. If one gets chosen the other must also be chosen
            }
            motorPriorities.get(bestIndex).update(); //Resetting the motor priority so that it knows that it updated the motor
            loopTime = (System.nanoTime() - loopStart) / 1000000000.0;
            a = bestScore; //Checking to make sure a motor was updated because if it wasn't then can move onto next thing
        }

        updateHub2 = false;
        loopStart = System.nanoTime();
    }
    public void getEncoders(){
        bulkData = expansionHub1.getBulkInputData();
        if (bulkData != null) {
            try {
                encoders[0] = bulkData.getMotorCurrentPosition(rightFront);
                encoders[1] = bulkData.getMotorCurrentPosition(leftFront);
                encoders[2] = bulkData.getMotorCurrentPosition(rightBack);
                localizer.updateEncoders(encoders);
                localizer.update();
                currentIntakeSpeed = ((double) bulkData.getMotorVelocity(leftBack)) / (((1.0+(46.0/11.0)) * 28.0) / (26.0/19.0));
                rightIntakeVal = bulkData.getAnalogInputValue(rightIntake);
                leftIntakeVal = bulkData.getAnalogInputValue(leftIntake);
                depositVal = bulkData.getAnalogInputValue(depositSensor);
                flexSensorVal = bulkData.getAnalogInputValue(flex);
            }
            catch (Exception e){
                Log.e("******* Error due to ",e.getClass().getName());
                e.printStackTrace();
                Log.e("******* fail", "control hub failed");
            }
        }
    }
    public void updateHub2(){
        if (!updateHub2) {
            updateHub2 = true;
            RevBulkData bulkData = expansionHub2.getBulkInputData();
            if (bulkData != null) {
                try {
                    currentSlideLength = bulkData.getMotorCurrentPosition(slides2) / 25.1372713591;
                    currentSlideSpeed = bulkData.getMotorVelocity(slides2) / 25.1372713591;
                    currentTurretAngle = bulkData.getMotorCurrentPosition(turret) / 578.3213;
                    distValLeft = bulkData.getAnalogInputValue(distLeft) / 3.2;
                    distValRight = bulkData.getAnalogInputValue(distRight) / 3.2;
                    magValLeft = bulkData.getAnalogInputValue(magLeft);
                    magValRight = bulkData.getAnalogInputValue(magRight);

                } catch (Exception e) {
                    Log.e("******* Error due to ", e.getClass().getName());
                    e.printStackTrace();
                    Log.e("******* fail", "expansion hub failed");
                }
            }
        }
    }
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftBack.setPower(v1);
        rightBack.setPower(v2);
        rightFront.setPower(v3);
    }


    //Lots of getters in order to ensure that the program updates the expansion hub
    public double getSlideLength(){
        updateHub2();
        return currentSlideLength;
    }
    public double getSlideSpeed(){
        updateHub2();
        return currentSlideSpeed;
    }
    public double getTurretAngle(){
        updateHub2();
        return currentTurretAngle;
    }
    public double getDistValLeft() {
        updateHub2();
        return distValLeft;
    }
    public double getDistValRight() {
        updateHub2();
        return distValRight;
    }
    public int getMagValLeft() {
        updateHub2();
        return magValLeft;
    }
    public int getMagValRight() {
        updateHub2();
        return magValRight;
    }
}
