package org.firstinspires.ftc.teamcode.Cheese;

import org.firstinspires.ftc.teamcode.AutoCode.AutoSupplies;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//opencv imports
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.AutoCode.AutoSupplies;

//webcam imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name="TestAltMoveFunc Auto", group="CompetitionAuto")

public class TestAltMoveFunc extends AutoSupplies {
    @Override
    public void runOpMode() {

        //  Establish all hardware
        initForAutonomous();
        //  Wait until start
        //waitForStart();
        initVision();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        //  ********************  Enter Code Here  ********************  \\
    }
}
