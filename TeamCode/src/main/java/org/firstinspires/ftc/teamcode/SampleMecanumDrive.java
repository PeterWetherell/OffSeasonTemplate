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

    int rightIntakeVal = 0, leftIntakeVal = 0, depositVal = 0, flexSensorVal = 0;
    double distValLeft = 0, distValRight = 0;int magValLeft = 0, magValRight = 0;

    double targetSlidesPose = 3, slidesSpeed = 1, slidesI = 0;
    double targetTurretPose = 0, turretI = 0;

    boolean updateHub2 = false;

    boolean startSlides = false;
    boolean startIntake = false;

    private long loopStart = System.nanoTime();
    double loopTime = 0.0;
    int loops = 0;

    public boolean transferMineral;
    public double currentIntake = 0;
    long currentTime = System.currentTimeMillis();
    long slideStart = currentTime, slideTime = currentTime, intakeTime = currentTime, startIntakeDepositTransfer = currentTime, startIntakeHit;
    long slidesDelay = currentTime, intakeDelay = currentTime, depositDelay = currentTime;
    public int intakeCase = 0, lastIntakeCase = 0;
    public int slidesCase = 0, lastSlidesCase = 0;
    boolean firstSlide = false;
    public int dropIntakeTime = 380;
    public double intakePower = -1;
    public int liftIntakeTime = 700;
    public int transfer1Time = 215;
    public int transfer2Time = 235;
    public double transfer1Power = 1.0;
    public int openDepositTime = 250;
    public int intakeLiftDelay = 100;
    public int effectiveDepositTime = openDepositTime;
    public double returnSlideLength = 0.35;
    long transferTime = System.currentTimeMillis();

    public static int intakeMinValRight = 200;
    public static int intakeMinValLeft = 100;
    public int numZeroLeft = 0;
    public int numZeroRight = 0;
    int numRightIntake = 0;
    int numLeftIntake = 0;

    double intakeTurretInterfaceHeading = Math.toRadians(57.5);
    public static double v4barInterfaceAngle = 0.15;
    public double depositAngle = Math.toRadians(-45);
    public double effectiveDepositAngle = Math.toRadians(-45);
    public static double depositInterfaceAngle = 0.8;
    public double depositTransferAngle = Math.toRadians(135);
    public double targetSlideExtensionLength = 0;
    public double targetTurretHeading = 0;
    public double targetV4barOrientation = 0;

    public boolean intakeDepositTransfer = false, intakeHit = false;

    public double leftIntakeDrop = 0.088;
    public double leftIntakeRaise = 0.79;
    public double leftIntakeMid = 0.721;
    public double rightIntakeDrop = 0.948;
    public double rightIntakeRaise = 0.279;
    public double rightIntakeMid = 0.356;

    double currentV4barAngle = 0;
    double targetV4barAngle = 0;
    double targetDepositAngle = 0;

    boolean deposit = false;

    public double turretOffset = 0;
    public double slidesOffset = 0;
    public double v4barOffset = 0;

    double currentDepositAngle = depositInterfaceAngle;

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
        transferMineral = false;
    }
    public void pinMotorPowers (double v, double v1, double v2, double v3) {
        motorPriorities.get(0).setTargetPower(v);
        motorPriorities.get(1).setTargetPower(v1);
        motorPriorities.get(2).setTargetPower(v2);
        motorPriorities.get(3).setTargetPower(v3);
    }
    public void update(){
        loops ++;

        getEncoders(); //This is the one thing that is guaranteed to occur every loop because we need encoders for odo

        if (System.currentTimeMillis() - intakeDelay >= 1000){
            startIntake = false;
        }
        if (System.currentTimeMillis() - depositDelay >= 1000){
            deposit = false;
        }
        if (System.currentTimeMillis() - slidesDelay >= 3000){
            startSlides = false;
        }
        if (startIntake && intakeCase == 0){
            intakeCase = 1;
            intakeTime = System.currentTimeMillis();
            startIntake = false;
        }
        if (startSlides && slidesCase == 0){
            slidesCase = 1;
            slideTime = System.currentTimeMillis();
            startSlides = false;
        }

        updateIntake();
        updateSlides();

        //Finds the correct value to set for the motor powers for the slides
        updateTurretHeading();
        updateSlidesLength();

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

    public void startDeposit(Pose2d endPose, Pose2d targetPose, double height, double radius){
        double turretX = -0.75;
        double depositLength = 4.0;
        endPose = new Pose2d(
                endPose.getX() + Math.cos(endPose.getHeading()) * turretX,
                endPose.getY() + Math.sin(endPose.getHeading()) * turretX,
                endPose.getHeading()
        );
        double d = Math.sqrt(Math.pow(targetPose.getX() - endPose.getX(),2) + Math.pow(targetPose.getY() - endPose.getY(),2));
        double x1 = targetPose.getX() + radius * -1.0 * (targetPose.getX()-endPose.getX())/d;
        double y1 = targetPose.getY() + radius * -1.0 * (targetPose.getY()-endPose.getY())/d;
        Pose2d relTarget = new Pose2d(
                Math.cos(endPose.getHeading())*(endPose.getX()-x1) + Math.sin(endPose.getHeading())*(endPose.getY()-y1),
                Math.cos(endPose.getHeading())*(endPose.getY()-y1) - Math.sin(endPose.getHeading())*(endPose.getX()-x1)
        );
        targetTurretHeading = Math.atan2(relTarget.getY(),relTarget.getX());
        height -= (9.44882 + Math.sin(depositAngle) * depositLength);
        double effectiveSlideAngle = Math.toRadians(8.92130165444);
        double v4BarLength = 8.75;
        double slope = Math.tan(effectiveSlideAngle);
        double length = Math.sqrt(Math.pow(relTarget.getY(),2) + Math.pow(relTarget.getX(),2)) - Math.cos(depositAngle) * depositLength - 7.9503937/Math.cos(effectiveSlideAngle);
        double a = (slope*slope + 1);
        double b = -1.0*(2*length + 2*slope*height);
        double c = length*length - Math.pow(v4BarLength,2) + height * height;
        if (4.0 * a * c < b * b) {
            double slideExtension = (-1.0 * b - Math.sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
            targetSlideExtensionLength = slideExtension / (Math.cos(effectiveSlideAngle));
            targetV4barOrientation = Math.atan2(height - slideExtension * slope, slideExtension - length);
        }
        else{
            targetSlideExtensionLength = length - v4BarLength;
            targetV4barOrientation = Math.toRadians(180);
        }
        while (targetV4barOrientation < 0){
            targetV4barOrientation += Math.PI * 2;
        }
        targetSlideExtensionLength = Math.max(0,targetSlideExtensionLength);
        startSlides = true;
        slidesDelay = System.currentTimeMillis();
    }

    public void startIntake(boolean rightIntake){
        double targetIntake = 1;
        if (rightIntake){
            targetIntake = -1;
        }
        if (currentIntake != targetIntake){
            currentIntake = targetIntake;
            setTurretTarget(intakeTurretInterfaceHeading * currentIntake);
        }
        startIntake = true;
        intakeDelay = System.currentTimeMillis();
    }


    public void updateIntake(){
        if (intakeHit && System.currentTimeMillis() - startIntakeHit > 500){
            intakeHit = false;
        }
        if (currentIntakeSpeed <= 16){
            intakeHit = true;
            startIntakeHit = System.currentTimeMillis();
        }
        if (rightIntakeVal >= intakeMinValRight) {
            numRightIntake ++;
            numZeroRight = 0;
        }
        else{
            numZeroRight ++;
        }
        if (numZeroRight >= 3){
            numRightIntake --;
        }
        numRightIntake = Math.max(0,Math.min(5,numRightIntake));

        if (leftIntakeVal >= intakeMinValLeft) {
            numLeftIntake ++;
            numZeroLeft = 0;
        }
        else{
            numZeroLeft ++;
        }
        if (numZeroLeft >= 3){
            numLeftIntake --;
        }
        numLeftIntake = Math.max(0,Math.min(5,numLeftIntake));

        if (!transferMineral){
            setDepositAngle(depositInterfaceAngle);
            setV4barOrientation(v4barInterfaceAngle);
            setTurretTarget(intakeTurretInterfaceHeading * currentIntake);
            if (Math.abs(getTurretAngle()) >= Math.toRadians(20)){
                setSlidesLength(returnSlideLength, 0.4);
            }
            else{
                setSlidesLength(returnSlideLength + 2.5, 0.4);
            }
            if (intakeCase == 0){
                if (currentIntake == 1){
                    servos.get(1).setPosition(leftIntakeRaise);
                }
                else {
                    servos.get(1).setPosition(leftIntakeMid);
                }
                if (currentIntake == -1){
                    servos.get(0).setPosition(rightIntakeRaise);
                }
                else {
                    servos.get(0).setPosition(rightIntakeMid);
                }
                motorPriorities.get(4).setTargetPower(0);
            }
        }
        if (lastIntakeCase != intakeCase) {
            switch (intakeCase) {
                case 1: motorPriorities.get(4).setTargetPower(0.3); break; // rotate the servo down
                case 2: motorPriorities.get(4).setTargetPower(intakePower); break; // turn on the intake (forward)
                case 3: transferTime = System.currentTimeMillis();break; // lift up the servo
                case 6:
                    Log.e("liftTime" , (System.currentTimeMillis() - transferTime) + "");
                    transferTime = System.currentTimeMillis();
                case 7:
                    motorPriorities.get(4).setTargetPower(transfer1Power);
                    break;
                case 8:
                    Log.e("transferTime" , (System.currentTimeMillis() - transferTime) + "");
                    motorPriorities.get(4).setTargetPower(0); transferMineral = true; intakeDepositTransfer = false;
                    setDepositAngle(depositInterfaceAngle + Math.toRadians(30)); //15
                    firstSlide = false;
                    break; // turn off the intake
            }
            intakeTime = System.currentTimeMillis();
        }
        lastIntakeCase = intakeCase;
        int a = intakeCase;
        switch (a) {
            case 1: case 2:
                if (intakeCase == 1 && System.currentTimeMillis() - intakeTime >= dropIntakeTime){intakeCase ++;}// waiting for the servo to drop
                if (intakeCase == 2 && ((currentIntake == -1 && numRightIntake >= 3) || (currentIntake == 1 && numLeftIntake >= 3)) && System.currentTimeMillis() - intakeTime >= 100){intakeCase ++;}

                if(currentIntake == 1){servos.get(1).setPosition(leftIntakeDrop);servos.get(0).setPosition(rightIntakeMid);}
                if(currentIntake == -1){servos.get(0).setPosition(rightIntakeDrop);servos.get(1).setPosition(leftIntakeMid);}
                break; // wait for block in
            case 3:
                if (System.currentTimeMillis() - intakeTime >= intakeLiftDelay) {
                    if (currentIntake == 1) {
                        servos.get(1).setPosition(leftIntakeRaise);
                    }
                    if (currentIntake == -1) {
                        servos.get(0).setPosition(rightIntakeRaise);
                    }
                }
                if ((((currentIntake == 1 && getMagValLeft() >= 1900) || (currentIntake == -1 && getMagValRight() >= 1900)) || System.currentTimeMillis() - intakeTime >= liftIntakeTime + intakeLiftDelay) && !transferMineral){
                    intakeCase ++;
                }
                if (!transferMineral){
                    setDepositAngle(depositInterfaceAngle);
                    setV4barOrientation(v4barInterfaceAngle);
                }
                break;  // waiting for the servo to go up && slides to be back 200 before
            case 4: if (Math.abs(getTurretAngle() - intakeTurretInterfaceHeading*currentIntake) <= Math.toRadians(7.5)){intakeCase ++;}break;//wait for the slides to be in the correct orientation
            case 5: if (Math.abs(targetV4barAngle - currentV4barAngle) < Math.toRadians(5) && Math.abs(getSlideLength() - returnSlideLength) < 0.5){intakeCase ++;}break;
            case 6: if (System.currentTimeMillis() - intakeTime >= 200 && (intakeDepositTransfer || System.currentTimeMillis() - intakeTime >= transfer1Time)){intakeCase ++;}break;
            case 7: if (System.currentTimeMillis() - intakeTime >= 30 && (intakeDepositTransfer || System.currentTimeMillis() - intakeTime >= transfer2Time)){intakeCase ++;currentDepositAngle = depositInterfaceAngle;}break;
        }
    }

    public void updateSlides(){
        if (intakeDepositTransfer && System.currentTimeMillis() - startIntakeDepositTransfer > 100){
            intakeDepositTransfer = false;
        }
        if (depositVal >= 15 || currentIntakeSpeed <= -18){
            intakeDepositTransfer = true;
            startIntakeDepositTransfer = System.currentTimeMillis();
        }

        if (transferMineral) { // I have deposited into the area
            if (lastSlidesCase != slidesCase) {
                slideTime = System.currentTimeMillis();
            }
            lastSlidesCase = slidesCase;
            int a = slidesCase;
            switch (a) {
                case 1: case 2: case 3:
                    double t = targetV4barOrientation + v4barOffset - Math.toRadians(10);
                    if (!firstSlide){
                        firstSlide = true;
                        slideStart = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - slideStart >= 80) {
                        setTurretTarget(targetTurretHeading + turretOffset);
                        double slidePower = 1.0;
                        double target;
                        double speed = 0.2;
                        double l = Math.abs(getSlideLength() - (targetSlideExtensionLength + slidesOffset));
                        if (targetSlideExtensionLength + slidesOffset <= 10) {
                            target = Math.toRadians(110);
                            slidePower = 0.5;
                        } else {
                            speed = 0.6; //1.1
                            target = Math.toRadians(107.5);
                        }
                        currentDepositAngle += Math.signum(target - currentDepositAngle) * Math.min(Math.abs((depositTransferAngle - depositInterfaceAngle) / speed) * loopTime, Math.toRadians(1.0));
                        if (Math.abs(target - currentDepositAngle) <= Math.toRadians(1)) {
                            currentDepositAngle = target;
                        }
                        if (Math.abs(getTurretAngle() - (targetTurretHeading + turretOffset)) <= Math.toRadians(10)) {
                            if (slidesCase == 1) {
                                setSlidesLength(4, slidePower);
                                setV4barOrientation(Math.min(Math.toRadians(137.1980907721663), t));
                            } else if (l < 10) {
                                setSlidesLength(targetSlideExtensionLength + slidesOffset, Math.max((slidePower - 0.65), 0.05) + Math.pow((targetSlideExtensionLength + slidesOffset - getSlideLength()) / 10.0, 2) * 0.25);//o.35
                                if (targetV4barOrientation + v4barOffset >= Math.toRadians(180) && Math.abs(getSlideSpeed()) >= 10 && Math.abs(currentV4barAngle - (targetV4barOrientation + v4barOffset - Math.toRadians(10))) >= Math.toRadians(5)) {
                                    setV4barOrientation(Math.min(Math.toRadians(137.1980907721663), t));
                                } else {
                                    setV4barOrientation(t);
                                }
                            } else {
                                setSlidesLength(targetSlideExtensionLength + slidesOffset, slidePower);
                                setV4barOrientation(Math.min(Math.toRadians(100), t));
                            }
                            setDepositAngle(currentDepositAngle);
                        }
                    }
                    else {
                        currentDepositAngle = Math.toRadians(105);
                        setDepositAngle(Math.toRadians(105));
                    }
                    if (slidesCase == 1 && ((Math.abs(getSlideLength() - 4) <= 3.5 && (currentV4barAngle >= Math.min(Math.toRadians(130),t))) || targetSlideExtensionLength + slidesOffset >= 10)){slidesCase ++;}
                    else if (slidesCase == 2 && (Math.abs(getTurretAngle() - (targetTurretHeading + turretOffset)) <= Math.toRadians(7.5)
                            && Math.abs(getSlideLength() - (targetSlideExtensionLength + slidesOffset)) <= 6
                            && Math.abs(t - currentV4barAngle) <= Math.toRadians(5))
                    ){slidesCase ++;}
                    if (slidesCase == 3 && deposit && System.currentTimeMillis() - slideTime >= 100){slidesCase ++; currentDepositAngle += Math.toRadians(20);setDepositAngle(Math.toRadians(180) - effectiveDepositAngle);updateDepositAngle();} //else if
                    break;
                case 4:
                    double depoAngle = Math.toRadians(180) - effectiveDepositAngle;
                    currentDepositAngle += Math.signum(depoAngle - currentDepositAngle) * (depoAngle - depositTransferAngle) / (0.1) * loopTime;
                    if (Math.abs(currentDepositAngle - depoAngle) <= Math.toRadians(2)){
                        currentDepositAngle =  depoAngle;
                    }

                    if (Math.abs(currentDepositAngle - depoAngle) <= Math.toRadians(10) && System.currentTimeMillis()-slideTime <= effectiveDepositTime - 150){
                        setV4barOrientation(targetV4barOrientation + v4barOffset);
                    }
                    else{
                        setV4barOrientation(targetV4barOrientation + v4barOffset - Math.toRadians(10));
                    }

                    setDepositAngle(depoAngle); //currentDepositAngle
                    setTurretTarget(targetTurretHeading + turretOffset);
                    setSlidesLength(targetSlideExtensionLength + slidesOffset,0.25);
                    if (slidesCase == 4 && System.currentTimeMillis() - slideTime >= effectiveDepositTime){slidesCase ++; intakeCase = 0; lastIntakeCase = 0; currentDepositAngle = depositTransferAngle;} // + 70 effectiveDepositTime
                    break;
                case 5 : case 6: case 7: case 8:
                    setDepositAngle(depositInterfaceAngle);
                    if (slidesCase == 5){
                        //DO NOTHING: NO MOVING SLIDES BACK NO MOVING V4BAR
                        if (System.currentTimeMillis() - slideTime >= 150){
                            setV4barOrientation(v4barInterfaceAngle);
                        }
                    }
                    else {
                        setV4barOrientation(v4barInterfaceAngle);
                        if (currentV4barAngle >= v4barInterfaceAngle + Math.toRadians(15)){
                            setSlidesLength(returnSlideLength + 3, 0.4);
                        }
                        else{
                            setSlidesLength(returnSlideLength, 0.6);
                            if (slidesCase == 8 && Math.abs(getSlideLength() - returnSlideLength) <= 2){
                                slidesCase ++;
                            }
                        }
                    }
                    if (slidesCase >= 7) {
                        setTurretTarget(intakeTurretInterfaceHeading * currentIntake);
                    }
                    else {
                        setTurretTarget(targetTurretHeading + turretOffset);
                    }
                    if (slidesCase == 5 && System.currentTimeMillis() - slideTime >= 250){slidesCase ++;} //400
                    else if (slidesCase == 6 && Math.abs(getSlideLength()-returnSlideLength) <= 4){slidesCase ++;}
                    else if (slidesCase == 7 && Math.abs(getTurretAngle() - intakeTurretInterfaceHeading*currentIntake) <= Math.toRadians(10)){slidesCase ++;}
                    break;
                case 9:
                    transferMineral = false; slidesCase = 0; lastSlidesCase = 0; deposit = false; effectiveDepositTime = openDepositTime;
                    break;
            }
        }
    }

    public void deposit(){
        deposit = true;
        depositDelay = System.currentTimeMillis();
    }

    public void setSlidesLength(double inches, double speed){
        targetSlidesPose = inches;
        slidesSpeed = speed;
    }

    public void updateSlidesLength(){
        double error = targetSlidesPose - getSlideLength();
        double p = error * 0.2;
        double kStatic = Math.signum(error) * slidesSpeed/2.0;
        if (Math.abs(error) <= 3 && loops >= 2){
            slidesI += (error) * loopTime * 0.05;
        }
        else{
            slidesI = 0;
        }
        if (Math.abs(error) <= 3){ // we are within 3 from the end
            p /= 2;
            if (error >= 0.5){ // the target is greater than the current by 0.5 we engage the PID to get it to 0
                kStatic = 0.175 + getSlideLength() * 0.002;
            }
            else if (error <= -0.5) { // the target is less than the current by 0.5 we slowly extend it back
                kStatic = -0.15 - (1.0/getSlideLength()) * 0.07;
                p /= 2.0;
            }
            else{// We are within +- 0.5 which means we use a holding power
                p = 0.085;
                kStatic = 0;
            }
        }
        else if (error < -3){ // We are more than 3 away so we go backward slowly
            kStatic = -0.1145; //-0.3 => 0.2 "ur bad" ~ @HudsonKim => 0.15
            p /= 4.0; // making it go back slower
        }
        else if (error > 3){ // We are more than 3 away so we go forward at max speed
            kStatic = slidesSpeed;
            p = 0;
        }
        motorPriorities.get(6).setTargetPower(kStatic + p + slidesI);
    }

    public void setTurretTarget(double radians){
        targetTurretPose = radians;
    }

    public void updateTurretHeading(){
        double error = targetTurretPose - getTurretAngle();
        turretI += Math.toDegrees(error) * loopTime * 0.01;
        if (Math.abs(error) >= Math.toRadians(5)){
            motorPriorities.get(5).setTargetPower(Math.signum(error) * 0.35 + turretI);
        }
        else if (Math.abs(error) >= Math.toRadians(1)){
            motorPriorities.get(5).setTargetPower(Math.signum(error) * 0.20 + turretI);
        }
        else {
            motorPriorities.get(5).setTargetPower(0);
            turretI = 0;
        }
    }

    public void setV4barDeposit(double targetDepositAngle, double targetV4barOrientation){
        targetV4barAngle = targetV4barOrientation;
        currentV4barAngle = targetV4barOrientation;
        this.targetDepositAngle = targetDepositAngle;
        updateDepositAngle();
        updateV4barAngle(0);
    }

    public void updateDepositAngle(){
        double angle = targetDepositAngle - currentV4barAngle;
        double targetPos = angle * 0.215820468 + 0.21;
        targetPos = Math.min(Math.max(targetPos,0.0),0.86);
        servos.get(2).setPosition(targetPos);
    }

    public void updateV4barAngle(double loopSpeed){
        currentV4barAngle += Math.signum(targetV4barAngle - currentV4barAngle) * Math.PI / 0.875 * loopSpeed; // 0.825 => 0.905
        if (Math.abs(targetV4barAngle - currentV4barAngle) < Math.toRadians(1)){
            currentV4barAngle = targetV4barAngle;
        }
        double servoPos = (targetV4barAngle * -0.201172) + 0.94;
        servoPos = Math.max(Math.min(servoPos,0.94),0.108); //0.94
        servos.get(4).setPosition(servoPos);
        updateDepositAngle();
    }

    public void setDepositAngle(double targetAngle){
        targetDepositAngle = targetAngle;
    }

    public void setV4barOrientation(double targetV4barOrientation){
        targetV4barAngle = targetV4barOrientation;
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
