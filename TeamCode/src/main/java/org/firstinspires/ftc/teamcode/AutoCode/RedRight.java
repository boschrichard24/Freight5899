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

@Autonomous(name="RedRight Auto", group="CompetitionAuto")
//@Disabled
public class RedRight extends AutoSupplies {
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

        int path = 0;

        Recognition duck = null;
        long halfSec = 700;
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

        //sleep(300);
        if (path == 1) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            setArmLevel(1);
        } else if (path == 2) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            setArmLevel(3);
        } else if(path == 3){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            setArmLevel(5);
        } else{ lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED); }
        //clawOpen();
        //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE);
        encoderMove(250,0.5,0.5);
        pause(1000);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE);
        turnToS(-90,0.7,3);
        pause(1000);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
        encoderMove(800,1,1);
        pause(1000);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_OCEAN_PALETTE);
        turnToS(225,0.5,3);
        pause(1000);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);


    }
}
