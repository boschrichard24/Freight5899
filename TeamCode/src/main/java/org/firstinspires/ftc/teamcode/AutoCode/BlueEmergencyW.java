package org.firstinspires.ftc.teamcode.AutoCode;
//-----imports-----
//main imports
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

@Autonomous(name="BlueEmergency Auto", group="CompetitionAuto")
//@Disabled
public class BlueEmergencyW extends AutoSupplies {
    @Override
    public void runOpMode() {

        //  Establish all hardware
        initForAutonomous();
        //  Wait until start
        waitForStart();
        initVision();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        int path = 0;

        Recognition duck = null;
        long halfSec = 2000;
        runtime.reset();
        while (runtime.milliseconds() <= halfSec) {
            duck = getDuckPosition();
            if (duck != null) {
                break;
            }
        }
        path = getZone(duck);
        telemetry.addLine("Path: " + path);
        telemetry.update();

        sleep(300);
        if (path == 1) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        } else if (path == 2) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (path == 3) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        clawOpen();
        encoderMove(500, .5, .7);
        turnToS(180, .6, 2);
        duckyMotorPower('B');
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE);
        turnToS(135, .6, 2);


        setDrivePower(0, 0);
        if (path == 2) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE);
            turnToS(90, .7, 2);
        } else if (path == 3) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
            turnToS(210, .7, 2);
        }
        else{
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
            turnToS(96, .4, 2);
        }
        setDrivePower(0,0);
        setArmLevel(3);
        sleep(1000);
        setArmLevel(1);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);
        sleep(400);
        clawClosed();
        sleep(200);
        setArmLevel(5);
        sleep(500);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SINELON);
        if(path == 3){
            turnToS(360, .7, 2);
            resetAngle();
        }
        else {
            turnToS(0, .5, 2);
        }
        if (path != 3) {
            turnToS(0, .6, 2);
            turnToS(0, .6, 2);
        } else {
            turnToS(0, .6, 2);
            turnToS(0, .6, 2);
        }

        setDrivePower(0,.3);
        sleep(800);//was not updated to the bot yet... was at 500
        setDrivePower(0,0);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
        for(int i = 0; i < 3; i++){
            clawOpen();
            sleep(500);
            clawClosed();
            sleep(500);
            clawOpen();
            sleep(100);
            clawClosed();
            sleep(100);
        }
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        setArmLevel(2);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        turnToS(0, .7, 2);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
        //  Turn all motors off and sleep
        setDrivePower(0, 0);
        sleep(1000);
    }
}
//Possible link to add voltage sensor into our code.
//https://www.reddit.com/r/FTC/comments/5cnilm/help_how_to_get_robot_battery_levelvoltage/

