package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.checkerframework.checker.units.qual.A;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class SampleMecanumDrive {

    double targetSlidesPose = 3, slidesSpeed = 1;
    double targetTurretPose = 0, turretI = 0;

    boolean startSlides = false;
    boolean startIntake = false;

    private long loopStart = System.nanoTime();
    double loopTime = 0.0;
    private long start = System.nanoTime();

    long currentTime = System.currentTimeMillis();
    long slideStart = currentTime, slideTime = currentTime, intakeTime = currentTime, startIntakeDepositTransfer = currentTime, startIntakeHit;
    long slidesDelay = currentTime, intakeDelay = currentTime, depositDelay = currentTime;

    public void setPose(double x, double y, double h) {
        localizer.setPose(x,y,h);
    }

    ExpansionHubEx expansionHub1, expansionHub2;
    public List<ExpansionHubMotor> motors;
    public ExpansionHubMotor leftFront, leftBack, rightBack, rightFront, intake, turret, slides, slides2;
    public ArrayList<UpdatePriority> motorPriorities = new ArrayList<>();
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

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides2.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront, intake, turret, slides, slides2);

        for (int i = 0; i < 4; i ++) {
            motors.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorPriorities.add(new UpdatePriority(3,5));
        }
        motorPriorities.add(new UpdatePriority(1,2));
        motorPriorities.add(new UpdatePriority(1,3));
        motorPriorities.add(new UpdatePriority(2,6));
    }
    public void setDriveMode(DcMotor.RunMode runMode){
        leftFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
        rightFront.setMode(runMode);
    }

    public ArrayList<Servo> servos;
    public CRServo duckSpin, duckSpin2;
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

    public AnalogInput rightIntake, leftIntake, depositSensor, distLeft, distRight, magLeft, magRight, flex;
    public VoltageSensor batteryVoltageSensor;
    public BNO055IMU imu;
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

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }

    public Localizer localizer;
    robotComponents r;
    private final FtcDashboard dashboard;
    public SampleMecanumDrive(HardwareMap hardwareMap){
        encoders = new int[3];
        initServos(hardwareMap);
        initMotors(hardwareMap);
        initSensors(hardwareMap);
        localizer = new Localizer();
        localizer.getIMU(imu);
        r = new robotComponents(true);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
    }
    int loops = 0;
    int sensorLoops = 0;
    int numMotorsUpdated = 0;
    public Pose2d target = null;
    boolean updateHub2 = false;

    PID turretPID = new PID(0.35,0.05,0.007);
    PID slidePID = new PID(0.2,0.05,0.001);
    public void update(){
        if (loops == 0){
            turretPID.update(0);
            slidePID.update(0);
            start = System.nanoTime();
            loopStart = System.nanoTime();
        }
        loops ++;
        getEncoders(); //This is the one thing that is guaranteed to occur every loop because we need encoders for odo

        loopTime = (System.nanoTime() - loopStart) / 1000000000.0; //gets the current time since the loop began
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("l loopSpeedBeforeMotors", loopTime * 1000);

        if (numMotorsUpdated == 0 || sensorLoops >= 4){
            sensorLoops = 0;
            updateHub2();
            updateIntake();
            updateSlides();

            double error = targetTurretPose - getTurretAngle();
            motorPriorities.get(5).setTargetPower(turretPID.update(error));

            error = targetSlidesPose - getSlideLength();
            double powerSlides;
            if (Math.abs(error) <= 3){
                if (Math.abs(error) >= 0.25){
                    powerSlides = slidePID.update(error);
                }
                else{
                    powerSlides = slidePID.update(0);
                }
            }
            else {
                slidePID.update(0);
                slidePID.resetIntegral();
                powerSlides = -0.2;
                if (error >= 3){
                    powerSlides = slidesSpeed;
                }
            }
            motorPriorities.get(6).setTargetPower(powerSlides);

        }
        else {
            sensorLoops ++;
        }

        double targetLoopLength = 0.004; //Sets the target loop time in seconds
        double bestMotorUpdate = 1;
        numMotorsUpdated = 0;
        while (sensorLoops != 0 && bestMotorUpdate > 0 && loopTime <= targetLoopLength) { // updates the motors while still time remaining in the loop
            int bestIndex = 0;
            bestMotorUpdate = motorPriorities.get(0).getPriority();
            for (int i = 1; i < motorPriorities.size(); i++) { //finding the motor that is most in need of being updated;
                if (motorPriorities.get(i).getPriority() > bestMotorUpdate) {
                    bestIndex = i;
                    bestMotorUpdate = motorPriorities.get(i).getPriority();
                }
            }
            if (bestMotorUpdate != 0) {
                numMotorsUpdated++;
                motors.get(bestIndex).setPower(motorPriorities.get(bestIndex).power); //setting the motor of the one that most needs it
                if (bestIndex == motorPriorities.size() - 1) {
                    numMotorsUpdated++;
                    slides2.setPower(motorPriorities.get(bestIndex).power); //This deals with the case of the linked mechanism for the slides. If one gets chosen the other must also be chosen
                }
                motorPriorities.get(bestIndex).update(); //Resetting the motor priority so that it knows that it updated the motor
            }
            loopTime = (System.nanoTime() - loopStart) / 1000000000.0;
        }

        updateHub2 = false;
        loopStart = System.nanoTime();

        updateV4barAngle(loopTime);

        packet.put("l loopSpeed", loopTime * 1000);
        packet.put("l avgLoopSpeed", (System.nanoTime() - start) / (1000000.0 * loops));
        packet.put("l numMotorsUpdated", numMotorsUpdated);

        packet.put("d/p X", currentPose.getX());
        packet.put("d/p Y", currentPose.getY());
        packet.put("d/p Heading (degrees)", Math.toDegrees(currentPose.getHeading()));
        packet.put("d/v relVelX", relCurrentVel.getX());
        packet.put("d/v relVelY", relCurrentVel.getY());
        packet.put("d/v relVelHeading (degrees)", Math.toDegrees(relCurrentVel.getHeading()));

        packet.put("s flexSensor", flexSensorVal);
        packet.put("r/t turretError (degrees)", Math.toDegrees(targetTurretPose - currentTurretAngle));
        packet.put("r/d depositVal", depositVal);
        packet.put("r/s slidesError", targetSlidesPose - currentSlideLength);
        packet.put("r/s slidesSpeed", currentSlideSpeed);
        packet.put("r/i intakeSpeed", currentIntakeSpeed);
        packet.put("r/i rightIntake", rightIntakeVal - intakeMinValRight);
        packet.put("r/i leftIntake", leftIntakeVal - intakeMinValLeft);
        packet.put("r/i/m rightMag", magValRight - 1900);
        packet.put("r/i/m leftMag", magValLeft - 1900);

        if (target != null) {
            packet.put("e relXError", (target.x - currentPose.x) * Math.cos(currentPose.heading) - (target.y - currentPose.y) * Math.sin(currentPose.heading));
            packet.put("e relYError", (target.x - currentPose.x) * Math.sin(currentPose.heading) + (target.y - currentPose.y) * Math.cos(currentPose.heading));
            double headingError = (target.heading + target.headingOffset) - currentPose.heading;
            while (headingError >= Math.PI) {
                headingError -= 2 * Math.PI;
            }
            while (headingError <= -Math.PI) {
                headingError += 2 * Math.PI;
            }
            packet.put("e headingError", Math.toDegrees(headingError));
        }

        Canvas fieldOverlay = packet.fieldOverlay();
        if (target != null) {
            packet.put("targetHeading", Math.toDegrees(target.heading));
            fieldOverlay.setStroke("#00FF00");
            fieldOverlay.strokeCircle(target.x, target.y, 2);
        }
        drawRobot(fieldOverlay,r,currentPose);
        //drawRobot(fieldOverlay,new robotComponents(true, "#FF69B4"),new Pose2d(localizer.xButNeg,localizer.yButNeg,localizer.heading));
        fieldOverlay.setStroke("#FF0000");
        fieldOverlay.strokeCircle(localizer.leftSensor.x,localizer.leftSensor.y, 2);
        fieldOverlay.strokeCircle(localizer.rightSensor.x,localizer.rightSensor.y, 2);
        dashboard.sendTelemetryPacket(packet);
    }

    int rightIntakeVal = 0, leftIntakeVal = 0, depositVal = 0, flexSensorVal = 0;
    int[] encoders;
    double currentIntakeSpeed = 0;
    RevBulkData bulkData;
    public Pose2d currentPose = new Pose2d(0,0,0);
    public Pose2d currentVel = new Pose2d(0,0,0);
    public Pose2d relCurrentVel = new Pose2d(0,0,0);
    public static int intakeMinValRight = 200;
    public static int intakeMinValLeft = 100;
    public int numZeroLeft = 0;
    public int numZeroRight = 0;
    int numRightIntake = 0;
    int numLeftIntake = 0;
    public boolean intakeDepositTransfer = false, intakeHit = false;
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

                localizer.wallUpdate(flexSensorVal);

                currentPose = localizer.currentPose;
                relCurrentVel = localizer.relCurrentVel;
                currentVel = localizer.currentVel;

                if (intakeDepositTransfer && System.currentTimeMillis() - startIntakeDepositTransfer > 100){
                    intakeDepositTransfer = false;
                }
                if (depositVal >= 15 || currentIntakeSpeed <= -18){
                    intakeDepositTransfer = true;
                    startIntakeDepositTransfer = System.currentTimeMillis();
                }

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
                    if (numZeroRight >= 3){
                        numRightIntake --;
                    }
                }
                numRightIntake = Math.max(0,Math.min(5,numRightIntake));

                if (leftIntakeVal >= intakeMinValLeft) {
                    numLeftIntake ++;
                    numZeroLeft = 0;
                }
                else{
                    numZeroLeft ++;
                    if (numZeroLeft >= 3){
                        numLeftIntake --;
                    }
                }
                numLeftIntake = Math.max(0,Math.min(5,numLeftIntake));
            }
            catch (Exception e){
                Log.e("******* Error due to ",e.getClass().getName());
                e.printStackTrace();
                Log.e("******* fail", "control hub failed");
            }
        }
    }
    double currentSlideLength = 0;
    double currentSlideSpeed = 0;
    double currentTurretAngle = 0;
    double distValLeft = 0, distValRight = 0;int magValLeft = 0, magValRight = 0;
    double lastDistValLeft = 0, lastDistValRight = 0;
    public void updateHub2(){
        if (!updateHub2) {
            updateHub2 = true;
            bulkData = expansionHub2.getBulkInputData();
            if (bulkData != null) {
                try {
                    currentSlideLength = bulkData.getMotorCurrentPosition(slides2) / 25.1372713591;
                    currentSlideSpeed = bulkData.getMotorVelocity(slides2) / 25.1372713591;
                    currentTurretAngle = bulkData.getMotorCurrentPosition(turret) / 578.3213;
                    distValLeft = bulkData.getAnalogInputValue(distLeft) / 3.2;
                    distValRight = bulkData.getAnalogInputValue(distRight) / 3.2;
                    magValLeft = bulkData.getAnalogInputValue(magLeft);
                    magValRight = bulkData.getAnalogInputValue(magRight);

                    if (lastDistValLeft != distValLeft || lastDistValRight != distValRight){
                        localizer.distUpdate(distValRight,distValLeft);
                    }
                    lastDistValLeft = distValLeft;
                    lastDistValRight = distValRight;
                } catch (Exception e) {
                    Log.e("******* Error due to ", e.getClass().getName());
                    e.printStackTrace();
                    Log.e("******* fail", "expansion hub failed");
                }
            }
        }
    }
    public boolean transferMineral = false;
    public double currentIntake = 0;
    public int intakeCase = 0, lastIntakeCase = 0;
    public int dropIntakeTime = 380;
    public double intakePower = -1;
    public int liftIntakeTime = 700;
    public int transfer1Time = 215;
    public int transfer2Time = 235;
    public double leftIntakeDrop = 0.088;
    public double leftIntakeRaise = 0.79;
    public double leftIntakeMid = 0.721;
    public double rightIntakeDrop = 0.948;
    public double rightIntakeRaise = 0.279;
    public double rightIntakeMid = 0.356;
    double intakeTurretInterfaceHeading = Math.toRadians(57.5);
    public static double depositInterfaceAngle = 0.8;
    public static double v4barInterfaceAngle = 0.15;
    long transferTime = System.currentTimeMillis();
    public int intakeLiftDelay = 100;
    public double transfer1Power = 1.0;
    public double returnSlideLength = 0.35;
    public void updateIntake(){
        if (startIntake && intakeCase == 0){
            intakeCase = 1;
            intakeTime = System.currentTimeMillis();
            startIntake = false;
        }
        if (System.currentTimeMillis() - intakeDelay >= 500){
            startIntake = false;
        }
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
                case 3: transferTime = System.currentTimeMillis();break; // lift up the servo
                case 6:
                    Log.e("liftTime" , (System.currentTimeMillis() - transferTime) + "");
                    transferTime = System.currentTimeMillis();
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
                motorPriorities.get(4).setTargetPower(0.3);
                if (a == 2){
                    motorPriorities.get(4).setTargetPower(intakePower);
                }
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
            case 6: motorPriorities.get(4).setTargetPower(transfer1Power); if (System.currentTimeMillis() - intakeTime >= 200 && (intakeDepositTransfer || System.currentTimeMillis() - intakeTime >= transfer1Time)){intakeCase ++;}break;
            case 7: motorPriorities.get(4).setTargetPower(transfer1Power); if (System.currentTimeMillis() - intakeTime >= 30 && (intakeDepositTransfer || System.currentTimeMillis() - intakeTime >= transfer2Time)){intakeCase ++;currentDepositAngle = depositInterfaceAngle;}break;
        }
    }
    public double turretOffset = 0;
    public double slidesOffset = 0;
    public double v4barOffset = 0;
    double currentDepositAngle = depositInterfaceAngle;
    public double depositTransferAngle = Math.toRadians(135);
    public double effectiveDepositAngle = Math.toRadians(-45);
    public int slidesCase = 0, lastSlidesCase = 0;
    boolean firstSlide = false;
    public int openDepositTime = 250;
    public int effectiveDepositTime = openDepositTime;
    public void updateSlides(){
        if (startSlides && slidesCase == 0){
            slidesCase = 1;
            slideTime = System.currentTimeMillis();
            startSlides = false;
        }
        if (System.currentTimeMillis() - depositDelay >= 500){
            deposit = false;
        }
        if (System.currentTimeMillis() - slidesDelay >= 500){
            startSlides = false;
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

                        double speed = 0.2;
                        double l = Math.abs(getSlideLength() - (targetSlideExtensionLength + slidesOffset));

                        double target;
                        if (targetSlideExtensionLength + slidesOffset <= 10) {
                            target = Math.toRadians(110);
                        } else {
                            speed = 0.6;
                            target = Math.toRadians(107.5);
                        }
                        currentDepositAngle += Math.signum(target - currentDepositAngle) * Math.min(Math.abs((depositTransferAngle - depositInterfaceAngle) / speed) * loopTime, Math.toRadians(1.0));
                        if (Math.abs(target - currentDepositAngle) <= Math.toRadians(1)) {
                            currentDepositAngle = target;
                        }
                        setDepositAngle(currentDepositAngle);

                        double slidePower = 1.0;
                        if (Math.abs(getTurretAngle() - (targetTurretHeading + turretOffset)) <= Math.toRadians(10)) {
                            if (slidesCase == 1) {
                                setSlidesLength(4, slidePower);
                                setV4barOrientation(Math.min(Math.toRadians(130), t));
                            } else if (l < 10) {
                                slidePower = 0.5;
                                setSlidesLength(targetSlideExtensionLength + slidesOffset, Math.max((slidePower - 0.65), 0.05) + Math.pow((targetSlideExtensionLength + slidesOffset - getSlideLength()) / 10.0, 2) * 0.25);//o.35
                                if (t >= Math.toRadians(160) && Math.abs(getSlideSpeed()) >= 10 && Math.abs(currentV4barAngle - t) >= Math.toRadians(5)) {
                                    setV4barOrientation(Math.min(Math.toRadians(137.1980907721663), t));
                                } else {
                                    setV4barOrientation(t);
                                }
                            } else {
                                setSlidesLength(targetSlideExtensionLength + slidesOffset, slidePower);
                                setV4barOrientation(Math.min(Math.toRadians(130), t));
                            }
                        }
                    }
                    else {
                        currentDepositAngle = Math.toRadians(105);
                        setDepositAngle(currentDepositAngle);
                    }
                    if (slidesCase == 1 && ((Math.abs(getSlideLength() - 4) <= 3.5 && (currentV4barAngle >= Math.min(Math.toRadians(130),t))) || targetSlideExtensionLength + slidesOffset >= 10)){slidesCase ++;}
                    else if (slidesCase == 2 && (Math.abs(getTurretAngle() - (targetTurretHeading + turretOffset)) <= Math.toRadians(7.5)
                            && Math.abs(getSlideLength() - (targetSlideExtensionLength + slidesOffset)) <= 6
                            && Math.abs(t - currentV4barAngle) <= Math.toRadians(5))
                    ){slidesCase ++;}
                    if (slidesCase == 3 && deposit && System.currentTimeMillis() - slideTime >= 100){slidesCase ++; setDepositAngle(Math.toRadians(180) - effectiveDepositAngle);updateDepositAngle();} //else if
                    break;
                case 4:
                    double depoAngle = Math.toRadians(180) - effectiveDepositAngle;
                    if (System.currentTimeMillis()-slideTime <= effectiveDepositTime - 150){
                        setV4barOrientation(targetV4barOrientation + v4barOffset);
                    }
                    else{
                        setV4barOrientation(targetV4barOrientation + v4barOffset - Math.toRadians(10));
                    }

                    setDepositAngle(depoAngle);
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
    double targetDepositAngle = 0;
    double currentV4barAngle = 0;
    double targetV4barAngle = 0;
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
    long lastLoop = System.nanoTime();
    public static PID heading = new PID(2.5,0.03,0.05);
    public static PID velX = new PID(0.1,0,0);
    public static PID velY = new PID(0.1,0,0);
    public static PID x = new PID(0.1,0,0);
    public static PID y = new PID(0.1,0,0);
    double lastT = 0;
    double lastF = 0;
    double lastL = 0;
    public void followTrajectory(LinearOpMode opMode, Trajectory trajectory){
        update();
        Pose2d targetPoint;
        lastLoop = System.nanoTime();
        trajectory.start();
        while (opMode.opModeIsActive() && trajectory.points.size() != 0){

            long currentTime = System.nanoTime();
            double loopTime = (lastLoop-currentTime)/1000000000.0;
            lastLoop = currentTime;

            update();
            trajectory.update(currentPose,relCurrentVel);
            targetPoint = trajectory.points.get(0);
            double error = Math.sqrt(Math.pow(currentPose.x - targetPoint.x,2) + Math.pow(currentPose.y - targetPoint.y,2));

            Pose2d lastTargetPoint = trajectory.points.get(trajectory.points.size()-1);
            double finalError = Math.sqrt(Math.pow(currentPose.x - lastTargetPoint.x,2) + Math.pow(currentPose.y - lastTargetPoint.y,2));

            double targetAngle = Math.atan2(targetPoint.y - currentPose.y,targetPoint.x - currentPose.x);
            double headingError = targetAngle - currentPose.heading;
            double relErrorX = Math.cos(headingError) * error;
            double relErrorY = Math.sin(headingError) * error;

            headingError += trajectory.points.get(0).headingOffset;

            if (finalError <= 8 && trajectory.points.size() < 100){
                headingError = lastTargetPoint.heading - currentPose.heading;
            }
            while (Math.abs(headingError) > Math.PI){
                headingError -= Math.PI * 2 * Math.signum(headingError);
            }

            double t = Math.signum(heading.update(headingError)-lastT) * Math.toRadians(120)/0.5 * loopTime + lastT; //Makes sure that it doesn't change target speed too quickly (turning)
            lastT = t;

            double f = 0;
            double l = 0;
            double errorSpeedButBad = Math.abs(relErrorY) + Math.abs(relErrorX);
            if (errorSpeedButBad != 0) {
                double speedConstant = targetPoint.speed * Math.min(1.1 - Math.abs(t),1.0) * 0.9 / errorSpeedButBad;
                f = Math.signum(velX.update(relErrorX * speedConstant * 55.0 - relCurrentVel.x)-lastF) * 20.0/0.5 * loopTime + lastF; //Makes sure that it doesn't change target speed too quickly (forward/back)
                l = Math.signum(velY.update(relErrorY * speedConstant * 40.0 - relCurrentVel.y)-lastL) * 15.0/0.5 * loopTime + lastL; //Makes sure that it doesn't change target speed too quickly (left/right)
                lastF = f;
                lastL = l;
            }

            //correct for horizontal error
            double errorX = (trajectory.pastPoints.get(0).x - currentPose.x)*Math.cos(currentPose.heading) - (trajectory.pastPoints.get(0).y - currentPose.y)*Math.sin(currentPose.heading);
            double errorY = (trajectory.pastPoints.get(0).x - currentPose.x)*Math.sin(currentPose.heading) + (trajectory.pastPoints.get(0).y - currentPose.y)*Math.cos(currentPose.heading);

            //trying to make it so that it only corrects for the amount that is perpendicular to the driving
            errorX *= Math.abs(Math.sin(trajectory.pastPoints.get(0).headingOffset));
            errorY *= Math.abs(Math.cos(trajectory.pastPoints.get(0).headingOffset));

            f += x.update(errorX);
            l += y.update(errorY);


            pinMotorPowers(f-l-t,f+l-t,f-l+t,f+l+t);
        }
        pinMotorPowers(0,0,0,0);
    }
    public void setSlidesLength(double inches, double speed) {
        targetSlidesPose = inches;
        slidesSpeed = speed;
    }
    public void setTurretTarget(double radians){
        targetTurretPose = radians;
    }
    public void setDepositAngle(double targetAngle){
        targetDepositAngle = targetAngle;
    }
    public void setV4barOrientation(double targetV4barOrientation){
        targetV4barAngle = targetV4barOrientation;
    }
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        localizer.updatePowerVector(new double[]{v,v1,v2,v3});
        leftFront.setPower(v);
        leftBack.setPower(v1);
        rightBack.setPower(v2);
        rightFront.setPower(v3);
    }
    public void pinMotorPowers (double v, double v1, double v2, double v3) {
        localizer.updatePowerVector(new double[]{v,v1,v2,v3});
        motorPriorities.get(0).setTargetPower(v);
        motorPriorities.get(1).setTargetPower(v1);
        motorPriorities.get(2).setTargetPower(v2);
        motorPriorities.get(3).setTargetPower(v3);
    }
    public void pinMotorPowers (double[] a) {
        localizer.updatePowerVector(a);
        motorPriorities.get(0).setTargetPower(a[0]);
        motorPriorities.get(1).setTargetPower(a[1]);
        motorPriorities.get(2).setTargetPower(a[2]);
        motorPriorities.get(3).setTargetPower(a[3]);
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
    public int getMagValLeft() {
        updateHub2();
        return magValLeft;
    }
    public int getMagValRight() {
        updateHub2();
        return magValRight;
    }
    public void startIntake(boolean rightIntake){
        double targetIntake = 1;
        if (rightIntake){
            targetIntake = -1;
        }
        startIntake = true;
        intakeDelay = System.currentTimeMillis();
    }
    public double targetSlideExtensionLength = 0;
    public double targetTurretHeading = 0;
    public double targetV4barOrientation = 0;
    public double depositAngle = Math.toRadians(-45);
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
        if (targetV4barOrientation < 0){
            targetV4barOrientation += Math.PI * 2;
        }
        targetSlideExtensionLength = Math.max(0,targetSlideExtensionLength);
        startSlides = true;
        slidesDelay = System.currentTimeMillis();
    }
    boolean deposit = false;
    public void deposit(){
        deposit = true;
        depositDelay = System.currentTimeMillis();
    }
    public void drawRobot(Canvas fieldOverlay, robotComponents r, Pose2d poseEstimate){
        for (Component c : r.components){
            fieldOverlay.setStrokeWidth(c.lineRadius);
            fieldOverlay.setStroke(c.color);
            if (c.p.size() == 1){
                drawPoint(fieldOverlay,c.p.get(0),c.radius,poseEstimate);
            }
            else {
                for (int i = 1; i < c.p.size() + 1; i++) {
                    drawPoint(fieldOverlay, c.p.get(i % c.p.size()), c.radius, poseEstimate);
                    drawLine(fieldOverlay, c.p.get(i % c.p.size()), c.p.get((i - 1)%c.p.size()), poseEstimate);
                }
            }
        }
    }
    public void drawLine(Canvas c, Point p, Point p2, Pose2d pose){
        double x1 = Math.cos(pose.getHeading())*(p.x+Math.signum(p.x-p2.x)*0.5) - Math.sin(pose.getHeading())*(p.y+Math.signum(p.y-p2.y)*0.5) + pose.getX();
        double y1 = Math.cos(pose.getHeading())*(p.y+Math.signum(p.y-p2.y)*0.5) + Math.sin(pose.getHeading())*(p.x+Math.signum(p.x-p2.x)*0.5) + pose.getY();
        double x2 = Math.cos(pose.getHeading())*(p2.x+Math.signum(p2.x-p.x)*0.5) - Math.sin(pose.getHeading())*(p2.y+Math.signum(p2.y-p.y)*0.5) + pose.getX();
        double y2 = Math.cos(pose.getHeading())*(p2.y+Math.signum(p2.y-p.y)*0.5) + Math.sin(pose.getHeading())*(p2.x+Math.signum(p2.x-p.x)*0.5) + pose.getY();
        c.strokeLine(x1,y1,x2,y2);
    }
    public void drawPoint(Canvas c, Point p, double radius, Pose2d pose){
        if (radius != 0){
            double x = Math.cos(pose.getHeading())*p.x - Math.sin(pose.getHeading())*p.y + pose.getX();
            double y = Math.cos(pose.getHeading())*p.y + Math.sin(pose.getHeading())*p.x + pose.getY();
            c.strokeCircle(x,y,radius);
        }
    }
}