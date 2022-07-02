package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@TeleOp
public class OdoDataGetter extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RevBulkData bulkData;
        ExpansionHubEx expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        ExpansionHubMotor rightFront = (ExpansionHubMotor) hardwareMap.dcMotor.get("rf");
        ExpansionHubMotor leftFront = (ExpansionHubMotor) hardwareMap.dcMotor.get("lf");
        ExpansionHubMotor rightBack = (ExpansionHubMotor) hardwareMap.dcMotor.get("rr");
        waitForStart();
        while (opModeIsActive()){
            bulkData = expansionHub1.getBulkInputData();
            double right = bulkData.getMotorCurrentPosition(rightFront);
            double left = bulkData.getMotorCurrentPosition(leftFront);
            double back = bulkData.getMotorCurrentPosition(rightBack);
            Log.e("OdoData", System.nanoTime() + "," + right + "," + left + "," + back);
        }
    }
}
