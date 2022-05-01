package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    public ExpansionHubMotor leftFront, leftRear, rightRear, rightFront, intake, turret, slides, slides2;

    public ArrayList<Servo> servos;
    public CRServo duckSpin, duckSpin2;

    public AnalogInput rightIntake, leftIntake, depositSensor, distLeft, distRight, magLeft, magRight, flex;
    public VoltageSensor batteryVoltageSensor;
    public ColorSensor leftWall, rightWall;
    public BNO055IMU imu;

    public void initMotors(HardwareMap hardwareMap){
        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        leftFront = (ExpansionHubMotor) hardwareMap.dcMotor.get("lf");
        leftRear = (ExpansionHubMotor) hardwareMap.dcMotor.get("lr");
        rightRear = (ExpansionHubMotor) hardwareMap.dcMotor.get("rr");
        rightFront = (ExpansionHubMotor) hardwareMap.dcMotor.get("rf");

        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        intake = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake");
        turret = (ExpansionHubMotor) hardwareMap.dcMotor.get("turret");
        slides = (ExpansionHubMotor) hardwareMap.dcMotor.get("slides");
        slides2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("slides2");

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront, intake, turret, slides, slides2);
    }

    public void setDriveMode(DcMotor.RunMode runMode){
        leftFront.setMode(runMode);
        leftRear.setMode(runMode);
        rightRear.setMode(runMode);
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
        initServos(hardwareMap);
        initMotors(hardwareMap);
        initSensors(hardwareMap);
    }

    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }
}
