package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class SampleMecanumDrive {

    double targetSlidesPose = 3, slidesSpeed = 1;
    double targetTurretPose = 0;

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

//        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
//        intake = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake");
//        turret = (ExpansionHubMotor) hardwareMap.dcMotor.get("turret");
//        slides = (ExpansionHubMotor) hardwareMap.dcMotor.get("slides");
//        slides2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("slides2");

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // resets odo readings
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // makes drive motors without encoder

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        slides.setDirection(DcMotorSimple.Direction.REVERSE);
//        slides2.setDirection(DcMotorSimple.Direction.REVERSE);
//        turret.setDirection(DcMotorSimple.Direction.REVERSE);

//        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront, intake, turret, slides, slides2);
        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);

        for (int i = 0; i < 4; i ++) {
            motors.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorPriorities.add(new UpdatePriority(motors.get(i),3,5));
        }
//        motorPriorities.add(new UpdatePriority(motors.get(4),1,2));
//        motorPriorities.add(new UpdatePriority(motors.get(5),1,3));
//        motorPriorities.add(new UpdatePriority(new ExpansionHubMotor[]{motors.get(6),motors.get(7)},2,6));
    }
    public void setDriveMode(DcMotor.RunMode runMode){
        leftFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
        rightFront.setMode(runMode);
    }

    public ArrayList<ExtraServo> servos;
    public CRServo duckSpin, duckSpin2;
    public void initServos(HardwareMap hardwareMap){
        servos = new ArrayList<>();
        servos.add(new ExtraServo(hardwareMap.servo.get("rightIntake"),"Super Speed",1));
        servos.add(new ExtraServo(hardwareMap.servo.get("leftIntake"),"Super Speed",1));
        servos.add(new ExtraServo(hardwareMap.servo.get("deposit"),"Super Speed",1)); servos.get(2).servoPositions(0.215820468,0.21,0,0.86);
        servos.add(new ExtraServo(hardwareMap.servo.get("odoLift"),"Torque",1));
        servos.add(new ExtraServo(hardwareMap.servo.get("v4bar"),"Torque",0.908333333333)); servos.get(4).servoPositions(-0.201172, 0.94,0.108,0.94);
        servos.add(new ExtraServo(hardwareMap.servo.get("rightCapstone"),"Torque",1));
        servos.add(new ExtraServo(hardwareMap.servo.get("leftCapstone"),"Torque",1));
        servos.add(new ExtraServo(hardwareMap.servo.get("duckSpinSpin"),"Torque",1));
        servos.add(new ExtraServo(hardwareMap.servo.get("rightOdo"),"Torque",1));
        servos.add(new ExtraServo(hardwareMap.servo.get("leftOdo"),"Torque",1));
        duckSpin = hardwareMap.crservo.get("duckSpin");
        duckSpin2 = hardwareMap.crservo.get("duckSpin2");
    }

    public AnalogInput rightIntake, leftIntake, depositSensor, magLeft, magRight, flex;

    public AnalogInput distLeft, distRight;

    public VoltageSensor batteryVoltageSensor;
    public BNO055IMU imu;
    private void initSensors(HardwareMap hardwareMap){
//        rightIntake = hardwareMap.analogInput.get("rightIntake");
//        leftIntake = hardwareMap.analogInput.get("leftIntake");
//        depositSensor = hardwareMap.analogInput.get("depositSensor");
//        magLeft = hardwareMap.analogInput.get("magLeft");
//        magRight = hardwareMap.analogInput.get("magRight");
//        flex = hardwareMap.analogInput.get("flex");
//
//        distLeft = hardwareMap.analogInput.get("distLeft");
//        distRight = hardwareMap.analogInput.get("distRight");
//
//        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
//
//        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public Localizer localizer;
    RobotComponents robotComponents;
    private final FtcDashboard dashboard;
    public SampleMecanumDrive(HardwareMap hardwareMap){
        encoders = new int[3];
//        initServos(hardwareMap);
        initSensors(hardwareMap);
        initMotors(hardwareMap);
        localizer = new Localizer();
        localizer.getIMU(imu);
        robotComponents = new RobotComponents(true); // for nice dashboard drawigngs
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
    }
    int loops = 0;
    int sensorLoops = 0;
    int numMotorsUpdated = 0;
    public TrajectoryPeice target = null;
    boolean updateHub2 = false;

    PID turretPID = new PID(0.35,0.05,0.007);
    PID slidePID = new PID(0.2,0.05,0.001);

    public void updateLoopTime(){
        loopTime = (System.nanoTime() - loopStart) / 1000000000.0; // converts from nano secs to secs
    }

    public void update(){
        if (loops == 0){
            turretPID.update(0);
            slidePID.update(0);
            start = System.nanoTime();
            loopStart = System.nanoTime();
        }
        loops ++;
        getEncoders(); //This is the one thing that is guaranteed to occur every loop because we need encoders for odo

        updateLoopTime(); // gets the current time since the loop began
        TelemetryPacket packet = new TelemetryPacket(); // for sending data to dashboard

        if (numMotorsUpdated == 0 || sensorLoops >= 4) { // motors have nothing to do or it's been 5 loops
            sensorLoops = 0;
            updateHub2();
            updateIntake();
            updateSlides();

            double error = targetTurretPose - getTurretAngle();
            motorPriorities.get(5).setTargetPower(turretPID.update(error));

            error = targetSlidesPose - getSlideLength();

            // Slides PID don't matter unless it is between 3 and 0.25 inches of error
            double powerSlides = 0;
            if (Math.abs(error) <= 3) {
                if (Math.abs(error) >= 0.25) {
                    powerSlides = slidePID.update(error);
                }
                else {
                    // need to update the PID so the PID knows loops are occurring and I term doesn't get huge
                    powerSlides = slidePID.update(0);
                }
            }
            else {
                slidePID.update(0);
                slidePID.resetIntegral();

                if (error >= 3){
                    powerSlides = slidesSpeed;
                }
            }

            if(error <= -3) {
                powerSlides = -0.2; // this will only trigger between -3 and -infinity.
            }

            motorPriorities.get(6).setTargetPower(powerSlides);
        }
        else {
            sensorLoops ++;
        }
        servos.get(2).offset = - servos.get(4).currentOrientation;

        double targetLoopLength = 0.010; //Sets the target loop time in seconds
        double bestMotorUpdate = 1;
        numMotorsUpdated = 0;
        updateLoopTime();
        double startMotorUpdate = loopTime;
        while (sensorLoops != 0 && bestMotorUpdate > 0 && loopTime <= targetLoopLength) { // updates the motors while still time remaining in the loop
            int bestIndex = 0;
            bestMotorUpdate = motorPriorities.get(0).getPriority(targetLoopLength - loopTime);

            // finds motor that needs updating the most
            for (int i = 1; i < motorPriorities.size(); i++) { //finding the motor that is most in need of being updated;
                double currentMotor = motorPriorities.get(i).getPriority(targetLoopLength - loopTime);
                if (currentMotor > bestMotorUpdate) {
                    bestIndex = i;
                    bestMotorUpdate = currentMotor;
                }
            }
            if (bestMotorUpdate != 0) { // priority # of motor needing update the most
                motorPriorities.get(bestIndex).update(); // Resetting the motor priority so that it knows that it updated the motor and setting the motor of the one that most needs it
                numMotorsUpdated += motorPriorities.get(bestIndex).motor.length; //adds the number of motors updated
            }
            updateLoopTime();
        }
        double totalMotorUpdateTime = loopTime - startMotorUpdate; // total time to update all the motors
        double averageTimeToUpdateMotor = totalMotorUpdateTime/Math.max((double)numMotorsUpdated,1.0); // time to update 1 motor, min must equal 1 so no divide by 0 error.

        updateHub2 = false; // sets to false, we've already gotten sensor data from hub
        loopStart = System.nanoTime();

        packet.put("l loopSpeed", loopTime * 1000);
        packet.put("l avgLoopSpeed", (System.nanoTime() - start) / (1000000.0 * loops));
        packet.put("l averageTimeToUpdateMotor", averageTimeToUpdateMotor);
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
            packet.put("e relYError", (target.x - currentPose.x) * Math.cos(currentPose.heading) - (target.y - currentPose.y) * Math.sin(currentPose.heading));
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
        drawRobot(fieldOverlay, robotComponents, currentPose);
        //drawRobot(fieldOverlay,new robotComponents(true, "#FF69B4"),new Pose2d(localizer.xButNeg,localizer.yButNeg,localizer.heading));
        fieldOverlay.setStroke("#FF0000");
        fieldOverlay.strokeCircle(localizer.leftSensor.x,localizer.leftSensor.y, 2);
        fieldOverlay.strokeCircle(localizer.rightSensor.x,localizer.rightSensor.y, 2);
        dashboard.sendTelemetryPacket(packet);
    }

    public Pose2d getRelError(Pose2d targetPoint){
        return new Pose2d((targetPoint.x - currentPose.x) * Math.cos(currentPose.heading) - (targetPoint.y - currentPose.y) * Math.sin(currentPose.heading),
                (targetPoint.x - currentPose.x) * Math.cos(currentPose.heading) - (targetPoint.y - currentPose.y) * Math.sin(currentPose.heading));
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

                currentIntakeSpeed = ((double) bulkData.getMotorVelocity(leftBack)) / (((1.0+(46.0/11.0)) * 28.0) / (26.0/19.0)); // rpm of intake
                rightIntakeVal = bulkData.getAnalogInputValue(rightIntake);
                leftIntakeVal = bulkData.getAnalogInputValue(leftIntake);
                depositVal = bulkData.getAnalogInputValue(depositSensor);
                flexSensorVal = bulkData.getAnalogInputValue(flex);

                localizer.wallUpdate(flexSensorVal);

                currentPose = localizer.currentPose;
                relCurrentVel = localizer.relCurrentVel;
                currentVel = localizer.currentVel;

                if (intakeDepositTransfer && System.currentTimeMillis() - startIntakeDepositTransfer > 100) { // 100ms backup timer for transfer
                    intakeDepositTransfer = false;
                }
                if (depositVal >= 15 || currentIntakeSpeed <= -18) { // transfer has been started
                    intakeDepositTransfer = true;
                    startIntakeDepositTransfer = System.currentTimeMillis();
                }

                if (intakeHit && System.currentTimeMillis() - startIntakeHit > 500) { // resets after 500ms
                    intakeHit = false;
                }
                if (currentIntakeSpeed <= 16){ // intake surgical tubing has slowed down after touching freight
                    intakeHit = true;
                    startIntakeHit = System.currentTimeMillis();
                }

                if (rightIntakeVal >= intakeMinValRight) { // freight detected in intake
                    numRightIntake ++;
                    numZeroRight = 0;
                }
                else { // freight not being detected in intake
                    numZeroRight ++;
                    if (numZeroRight >= 3) { // waits for three loops before decreasing number
                        numRightIntake --;
                    }
                }
                numRightIntake = Math.max(0,Math.min(5,numRightIntake)); // numRightIntake must be between 0 and 5.

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
            catch (Exception e) { // catches any errors from reading data to avoid crashing in middle of program
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
    public void updateHub2() {
        if (!updateHub2) {
            updateHub2 = true;
            bulkData = expansionHub2.getBulkInputData();
            if (bulkData != null) {
                try {
                    currentSlideLength = bulkData.getMotorCurrentPosition(slides2) / 25.1372713591;
                    currentSlideSpeed = bulkData.getMotorVelocity(slides2) / 25.1372713591;
                    currentTurretAngle = bulkData.getMotorCurrentPosition(turret) / 578.3213;
                    magValLeft = bulkData.getAnalogInputValue(magLeft);
                    magValRight = bulkData.getAnalogInputValue(magRight);

                    distValLeft = bulkData.getAnalogInputValue(distLeft) / 3.2;
                    distValRight = bulkData.getAnalogInputValue(distRight) / 3.2;

                    if (lastDistValLeft != distValLeft || lastDistValRight != distValRight){
                        localizer.distUpdate(distValRight,distValLeft);
                    }
                    lastDistValLeft = distValLeft;
                    lastDistValRight = distValRight;
                } catch (Exception e) { // catches any errors from reading data in to avoid crashing program
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
    public double intakeTurretInterfaceHeading = Math.toRadians(57.5);
    public static double depositInterfaceAngle = 0.8; // ~45 degrees
    public static double v4barInterfaceAngle = 0.15;
    long transferTime = System.currentTimeMillis();
    public int intakeLiftDelay = 100;
    public double transfer1Power = 1.0;
    public double returnSlideLength = 0.35;

    // 0 idle
    // 1 drop intake
    // 2 waiting for freight in intake
    // 3 lift intake
    // 4 turret in right position
    // 5 slides in right position
    // 6 hard transfer (fast)
    // 7 slow transfer (slow)
    // 8 reset intake and wait for slides to reset and then go to idle

    public void updateIntake(){
        if (startIntake && intakeCase == 0) { // going from idle to starting intake
            intakeCase = 1;
            intakeTime = System.currentTimeMillis();
            startIntake = false;
        }
        if (System.currentTimeMillis() - intakeDelay >= 500) {
            startIntake = false;
        }
        if (!transferMineral) { // if not transferring mineral
            servos.get(2).setOrientation(depositInterfaceAngle);
            servos.get(4).setOrientation(v4barInterfaceAngle);
            setTurretTarget(intakeTurretInterfaceHeading * currentIntake); // move turret to left(1) or right(-1) side
            if (Math.abs(getTurretAngle()) >= Math.toRadians(20)) { // notice absolute
                setSlidesLength(returnSlideLength, 0.4);
            }
            else{
                setSlidesLength(returnSlideLength + 2.5, 0.4); // so doesn't get caught on transfer guide
            }
            if (intakeCase == 0) { // intake still, setting left and right intake's servo pos
                if (currentIntake == 1) { // left
                    servos.get(1).setPosition(leftIntakeRaise);
                }
                else {
                    servos.get(1).setPosition(leftIntakeMid);
                }
                if (currentIntake == -1) { // right
                    servos.get(0).setPosition(rightIntakeRaise);
                }
                else {
                    servos.get(0).setPosition(rightIntakeMid);
                }
                motorPriorities.get(4).setTargetPower(0);
            }
        }
        if (lastIntakeCase != intakeCase) { // change in case
            // most for logging
            switch (intakeCase) {
                case 3:
                    transferTime = System.currentTimeMillis();
                    break; // lift up the servo
                case 6:
                    Log.e("liftTime" , (System.currentTimeMillis() - transferTime) + "");
                    transferTime = System.currentTimeMillis();
                    break;
                case 8: // reset
                    Log.e("transferTime" , (System.currentTimeMillis() - transferTime) + "");
                    motorPriorities.get(4).setTargetPower(0);
                    transferMineral = true;
                    intakeDepositTransfer = false;
                    servos.get(2).setOrientation(depositInterfaceAngle + Math.toRadians(30)); // makes sure deposit bucket doesn't hit hub
                    firstSlide = false;
                    break; // turn off the intake
            }
            intakeTime = System.currentTimeMillis(); // time since last change in case
        }
        lastIntakeCase = intakeCase;
        int a = intakeCase;
        switch (a) {
            case 1: case 2:
                motorPriorities.get(4).setTargetPower(0.3); // start intake as dropping intake
                if (a == 2) {
                    motorPriorities.get(4).setTargetPower(intakePower); // -1 power
                }
                if (intakeCase == 1 && System.currentTimeMillis() - intakeTime >= dropIntakeTime){intakeCase ++;}// waiting for the servo to drop
                // line below checks left/right intake and if force sensor inside has been triggered, also makes sure that it doesn't happen within the first 100ms since intake drop
                if (intakeCase == 2 && ((currentIntake == -1 && numRightIntake >= 3) || (currentIntake == 1 && numLeftIntake >= 3)) && System.currentTimeMillis() - intakeTime >= 100){intakeCase ++;}

                if(currentIntake == 1){servos.get(1).setPosition(leftIntakeDrop);servos.get(0).setPosition(rightIntakeMid);} // drop left intake
                if(currentIntake == -1){servos.get(0).setPosition(rightIntakeDrop);servos.get(1).setPosition(leftIntakeMid);} // drop right intake
                break; // wait for block in
            case 3:
                if (System.currentTimeMillis() - intakeTime >= intakeLiftDelay) { // waits intakeLiftDelay (100ms) before lifting intake
                    if (currentIntake == 1) {
                        servos.get(1).setPosition(leftIntakeRaise);
                    }
                    if (currentIntake == -1) {
                        servos.get(0).setPosition(rightIntakeRaise);
                    }
                }
                // checks that either magnets get triggered or time based last resort is triggered
                if ((((currentIntake == 1 && getMagValLeft() >= 1900) || (currentIntake == -1 && getMagValRight() >= 1900)) || System.currentTimeMillis() - intakeTime >= liftIntakeTime + intakeLiftDelay) && !transferMineral){
                    intakeCase ++;
                }
                if (!transferMineral){
                    servos.get(2).setOrientation(depositInterfaceAngle);
                    servos.get(4).setOrientation(v4barInterfaceAngle);
                }
                break;  // waiting for the servo to go up && slides to be back 200 before
            case 4: if (Math.abs(getTurretAngle() - intakeTurretInterfaceHeading*currentIntake) <= Math.toRadians(7.5)){intakeCase ++;}break;// waits for the turret to be in the correct orientation within 7.5 deg
            case 5: if (Math.abs(v4barInterfaceAngle - servos.get(4).getOrientation()) < Math.toRadians(5) && Math.abs(getSlideLength() - returnSlideLength) < 0.5){intakeCase ++;}break; // waits for v4bar and slides to be in correct orientation within some margin
            // sets power for intake, moves onto next case if time since case 5 is > 200 ms and either the intakeDepositTransfer = true or time since case 5 is > transfer1Time (215 ms)
            case 6: motorPriorities.get(4).setTargetPower(transfer1Power); if (System.currentTimeMillis() - intakeTime >= 200 && (intakeDepositTransfer || System.currentTimeMillis() - intakeTime >= transfer1Time)){intakeCase ++;}break;
            // sets power for intake, moves onto next case if time since case 6 is > 30 and either the intakeDepositTransfer = true or time since case 6 is > transfer2Time
            // also sets v4bar for deposit angle
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

    // 0 idle
    // 1 shared
    // 2 turret, slides, v4bar are all correct length
    // 3 deposit has been triggered
    // 4 certain amount of time has passed such that the freight has come out of the deposit
    // 5 moves v4bar for 250 ms
    // 6 slides return
    // 7 turret is correct
    // 8 slides are fully all the way in
    // 9 reset everything and go to idle

    public void updateSlides(){
        if (startSlides && slidesCase == 0){
            slidesCase = 1;
            slideTime = System.currentTimeMillis();
            startSlides = false;
        }

        // timer backups
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
                    double t = depositTargetV4barOrientation + v4barOffset - Math.toRadians(10); // v4bar target angle
                    if (!firstSlide) {
                        firstSlide = true;
                        slideStart = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - slideStart >= 80) {

                        setTurretTarget(depositTargetTurretHeading + turretOffset);

                        double l = Math.abs(getSlideLength() - (depositTargetSlideExtensionLength + slidesOffset)); // slides error

                        double speed = 0.41;
                        double target = Math.toRadians(107.5);
                        if (depositTargetSlideExtensionLength + slidesOffset >= 10) {
                            target = Math.toRadians(110);
                            speed = 0.14;
                        }
                        servos.get(2).setOrientation(target,speed);

                        double slidePower = 1.0;
                        if (Math.abs(getTurretAngle() - (depositTargetTurretHeading + turretOffset)) <= Math.toRadians(10)) {
                            if (slidesCase == 1) { // shared
                                setSlidesLength(4, slidePower);
                                servos.get(4).setOrientation(Math.min(Math.toRadians(130), t));
                            } else if (l < 10) { // alliance, slides are extended within 10 inches of target --> slow down
                                slidePower = 0.5;
                                setSlidesLength(depositTargetSlideExtensionLength + slidesOffset, Math.max((slidePower - 0.65), 0.05) + Math.pow((depositTargetSlideExtensionLength + slidesOffset - getSlideLength()) / 10.0, 2) * 0.25);//o.35
                                if (t >= Math.toRadians(160) && Math.abs(getSlideSpeed()) >= 10 && Math.abs(servos.get(4).getOrientation() - t) >= Math.toRadians(5)) { // avoid hitting hub?
                                    servos.get(4).setOrientation(Math.min(Math.toRadians(137.1980907721663), t));
                                } else {
                                    servos.get(4).setOrientation(t);
                                }
                            } else {
                                setSlidesLength(depositTargetSlideExtensionLength + slidesOffset, slidePower);
                                servos.get(4).setOrientation(Math.min(Math.toRadians(130), t));
                            }
                        }
                    }
                    else {
                        servos.get(2).setOrientation(Math.toRadians(105));
                    }
                    // case 1 is shared. This if statement moves from shared to next case if slides and v4bar are back or slides have extended past a certain point
                    if (slidesCase == 1 && ((Math.abs(getSlideLength() - 4) <= 3.5 && (servos.get(4).getOrientation() >= Math.min(Math.toRadians(130),t))) || depositTargetSlideExtensionLength + slidesOffset >= 10)){slidesCase ++;}
                    // makes sure turret, slides, v4bar are all correct length within some margin
                    else if (slidesCase == 2 && (Math.abs(getTurretAngle() - (depositTargetTurretHeading + turretOffset)) <= Math.toRadians(7.5)
                            && Math.abs(getSlideLength() - (depositTargetSlideExtensionLength + slidesOffset)) <= 6
                            && Math.abs(t - servos.get(4).getOrientation()) <= Math.toRadians(5))
                    ){slidesCase ++;}
                    // deposit has been triggered and at least 100 ms has passed since last case. Resets deposit angle after finished depositing
                    if (slidesCase == 3 && deposit && System.currentTimeMillis() - slideTime >= 100){slidesCase ++;} //else if
                    break;
                case 4:
                    if (System.currentTimeMillis()-slideTime <= effectiveDepositTime - 150) { // waiting for freight to fall out of deposit
                        servos.get(4).setOrientation(depositTargetV4barOrientation + v4barOffset);
                    }
                    else{ // freight has fallen out of deposit
                        servos.get(4).setOrientation(depositTargetV4barOrientation + v4barOffset - Math.toRadians(10));
                    }

                    servos.get(2).setOrientation(Math.toRadians(180) - effectiveDepositAngle);
                    setTurretTarget(depositTargetTurretHeading + turretOffset);
                    setSlidesLength(depositTargetSlideExtensionLength + slidesOffset,0.25);
                    // moves onto next case and resets. Sets depositAngle to prepare for transfer
                    if (slidesCase == 4 && System.currentTimeMillis() - slideTime >= effectiveDepositTime){slidesCase ++; intakeCase = 0; lastIntakeCase = 0; currentDepositAngle = depositTransferAngle;} // + 70 effectiveDepositTime
                    break;
                case 5 : case 6: case 7: case 8:
                    servos.get(2).setOrientation(depositInterfaceAngle);
                    if (slidesCase > 5 || System.currentTimeMillis() - slideTime >= 150){
                        servos.get(4).setOrientation(v4barInterfaceAngle);
                    }
                    if (slidesCase > 5) { // slides have already retracted
                        if (servos.get(4).getOrientation() >= v4barInterfaceAngle + Math.toRadians(15)){
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
                        setTurretTarget(depositTargetTurretHeading + turretOffset);
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
    public void depositAtPoint(LinearOpMode opMode, Pose2d end){
        target = new TrajectoryPeice(end,0,0);
        double side = Math.signum(end.y);
        while (opMode.opModeIsActive() && slidesCase <= 4) {
            startDeposit(new Pose2d(currentPose.getX(),currentPose.getY(),currentPose.getHeading()), new Pose2d(-13.0, 24.0 * side),13.5,3);
            deposit();
            update();
            Pose2d error = getRelError(end);
            double a = 0.10 * side; //0.15
            if (Math.abs(error.getY()) <= 0.5 || slidesCase == 4){ //0.5
                error = new Pose2d(error.getX(), 0, error.getHeading());
                a *= 3.5;
            }
            if (slidesCase == 4){
                startIntake(side == -1);
            }
            error.heading = 0 - currentPose.heading;

            double f = error.x * 0.2;
            double l = error.y * 0.2 + a;
            double t = Math.toDegrees(error.heading) * 0.04;
            pinMotorPowers(f-l-t,f+l-t,f-l+t,f+l+t);
        }
        target = null;
    }

    public void intakeMineral(LinearOpMode opMode, double power, long maxTime, boolean goBackIfStall){
        long startingTime = System.currentTimeMillis();
        Pose2d lastPose = currentPose;
        long lastGoodIntake = System.currentTimeMillis();
        long stall = System.currentTimeMillis();
        long noStall = System.currentTimeMillis();
        long a = System.currentTimeMillis();
        boolean first = true;
        boolean c = true;
        double kI = 0;
        double side = Math.signum(currentPose.y);
        while(opMode.opModeIsActive() && intakeCase <= 2 && a-startingTime <= maxTime){
            double sidePower = 0;
            a = System.currentTimeMillis();
            if (a - startingTime >= 150 && Math.abs(relCurrentVel.getX()) <= 2 && goBackIfStall){
                if (first && a - lastGoodIntake >= 300) {
                    first = false;
                    maxTime += 500;
                    long goBack = System.currentTimeMillis();
                    while (opMode.opModeIsActive() && intakeCase <= 2 && System.currentTimeMillis()-goBack <= 500){
                        update();
                        pinMotorPowers(-0.2,-0.2,-0.2,-0.2);
                    }
                }
            }
            else{
                lastGoodIntake = a;
                lastPose = currentPose;
            }

            if (Math.abs(currentIntakeSpeed) <= 3 && a - noStall >= 200 && c){
                motorPriorities.get(4).setTargetPower(transfer1Power);
                stall = a;
                c = false;
                first = false;
                maxTime += 500;
                while (opMode.opModeIsActive() && System.currentTimeMillis() - a <= 100){
                    update();
                }
                motorPriorities.get(4).setTargetPower(intakePower);
                Pose2d q = new Pose2d(lastPose.getX() - 2, lastPose.getY() - 4 * side, lastPose.getHeading() - Math.toRadians(8 * side));
                driveToPoint(opMode, q, q, true, 1, 1000, false);
            }

            if (a - stall >= 100 && !c){
                motorPriorities.get(4).setTargetPower(intakePower);
                c = true;
                noStall = a;
            }


            double turn = 0;
            double targetSpeed = power * 30;
            double currentSpeed = Math.abs(relCurrentVel.getX());
            double speedError = targetSpeed-currentSpeed;
            double kP = speedError * 0.02;
            kI += speedError * loopTime * 0.0005;
            double multiplier = Math.min(1.0/(Math.abs(power) + Math.abs(turn) + Math.abs(sidePower)),1);
            double f = kP + kI + power * 0.75;
            pinMotorPowers((f+turn-sidePower)*multiplier,(f+turn+sidePower)*multiplier,(f-turn-sidePower)*multiplier,(f-turn+sidePower)*multiplier);
            update();
        }
        motorPriorities.get(4).setTargetPower(-1);
        pinMotorPowers(0,0,0,0);
    }

    public void driveToPoint(LinearOpMode opMode, Pose2d target, Pose2d target2, boolean intake, double error, long maxTime, boolean forward){
        this.target = new TrajectoryPeice(target,0,0);
        long startH = System.currentTimeMillis();
        update();
        boolean x = (Math.max(target.getX(),target2.getX()) + error > currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < currentPose.getX());
        boolean y = (Math.max(target.getY(),target2.getY()) + error > currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < currentPose.getY());
        while (
                opMode.opModeIsActive() && (!forward || currentPose.getX() <= target.getX() - error)
                        && !(x && y &&  Math.abs(currentPose.getHeading() - target.getHeading()) < Math.toRadians(error * 2))
                        && (((intakeCase <= 2 || intakeCase == 8) || currentPose.getX() <= 38) || !intake) && System.currentTimeMillis() - startH < maxTime //39
        ) {
            update();
            x = (Math.max(target.getX(),target2.getX()) + error > currentPose.getX() && Math.min(target.getX(),target2.getX()) - error < currentPose.getX());
            y = (Math.max(target.getY(),target2.getY()) + error > currentPose.getY() && Math.min(target.getY(),target2.getY()) - error < currentPose.getY());
            Pose2d relError = getRelError(target);
            relError.heading = target.heading - currentPose.heading;
            double f = (Math.min(Math.abs(relError.x),5) * 0.1 + 0.20) * Math.signum(relError.x);
            double l = (Math.min(Math.abs(relError.y),5) * 0.1 + 0.20) * Math.signum(relError.y);
            double t = Math.toDegrees(relError.heading) * 0.04;
            pinMotorPowers(f-l-t,f+l-t,f-l+t,f+l+t);
        }
        this.target = null;
    }
    long lastLoop = System.nanoTime();
    public static PID heading = new PID(2.5,0.03,0.05);
    public static PID velX = new PID(0.1,0,0);
    public static PID velY = new PID(0.1,0,0);
    double lastT = 0;
    double lastF = 0;
    double lastL = 0;
    public void followTrajectory(LinearOpMode opMode, Trajectory trajectory, boolean continueAfterTime){
        update();
        lastLoop = System.nanoTime();
        trajectory.start();
        long gameOver = System.currentTimeMillis();
        while ((opMode.opModeIsActive() || (continueAfterTime && System.currentTimeMillis() - gameOver <= 900)) && trajectory.points.size() != 0 && (((intakeCase <= 2 || intakeCase == 8) || currentPose.getX() <= 38) || !trajectory.points.get(0).intakeInterupt)){
            if (opMode.opModeIsActive()){
                gameOver = System.currentTimeMillis();
            }
            long currentTime = System.nanoTime();
            double loopTime = (lastLoop-currentTime)/1000000000.0;
            lastLoop = currentTime;

            update();
            Pose2d relError = trajectory.update(currentPose,relCurrentVel);
            target = trajectory.points.get(0);

            double t = Math.signum(heading.update(relError.heading)-lastT) * Math.toRadians(120)/0.5 * loopTime + lastT; //Makes sure that it doesn't change target speed too quickly (turning)
            lastT = t;

            double f = 0;
            double l = 0;
            double errorSpeedButBad = Math.abs(relError.y) + Math.abs(relError.x);
            if (errorSpeedButBad != 0) {
                double speedConstant = trajectory.points.get(0).speed * Math.min(1.1 - Math.abs(t),1.0) * 0.9 / errorSpeedButBad;
                f = Math.signum(velX.update(relError.x * speedConstant * 55.0 - relCurrentVel.x)-lastF) * 15.0/0.5 * loopTime + lastF; //Makes sure that it doesn't change target speed too quickly (forward/back)
                l = Math.signum(velY.update(relError.y * speedConstant * 40.0 - relCurrentVel.y)-lastL) * 12.5/0.5 * loopTime + lastL; //Makes sure that it doesn't change target speed too quickly (left/right)
                lastF = f;
                lastL = l;
            }

            pinMotorPowers(f-l-t,f+l-t,f-l+t,f+l+t);
        }
        pinMotorPowers(0,0,0,0);
    }
    public void followTrajectory(LinearOpMode opMode, Trajectory trajectory) {
        followTrajectory(opMode,trajectory,false);
    }
    public void setSlidesLength(double inches, double speed) {
        targetSlidesPose = inches;
        slidesSpeed = speed;
    }
    public void setTurretTarget(double radians){
        targetTurretPose = radians;
    }
    public void setMotorPowers(double v, double v1, double v2, double v3) { // set motor power (only use in test programs)
        localizer.updatePowerVector(new double[]{v,v1,v2,v3});
        leftFront.setPower(v);
        leftBack.setPower(v1);
        rightBack.setPower(v2);
        rightFront.setPower(v3);
    }
    public void pinMotorPowers (double v, double v1, double v2, double v3) { // sets motor priorities (use in actual programs for comp)
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
        double targetIntake = 1; // left intake
        if (rightIntake){
            targetIntake = -1; // right intake
        }
        startIntake = true;
        intakeDelay = System.currentTimeMillis();
    }
    public double depositTargetSlideExtensionLength = 0;
    public double depositTargetTurretHeading = 0;
    public double depositTargetV4barOrientation = 0;
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
        depositTargetTurretHeading = Math.atan2(relTarget.getY(),relTarget.getX());
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
            depositTargetSlideExtensionLength = slideExtension / (Math.cos(effectiveSlideAngle));
            depositTargetV4barOrientation = Math.atan2(height - slideExtension * slope, slideExtension - length);
        }
        else{
            depositTargetSlideExtensionLength = length - v4BarLength;
            depositTargetV4barOrientation = Math.toRadians(180);
        }
        if (depositTargetV4barOrientation < 0){
            depositTargetV4barOrientation += Math.PI * 2;
        }
        depositTargetSlideExtensionLength = Math.max(0,depositTargetSlideExtensionLength);
        startSlides = true;
        slidesDelay = System.currentTimeMillis();
    }
    boolean deposit = false;
    public void deposit(){
        deposit = true;
        depositDelay = System.currentTimeMillis();
    }
    public void drawRobot(Canvas fieldOverlay, RobotComponents r, Pose2d poseEstimate){
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