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

@Autonomous(name="SpitOut", group="CompetitionAuto")
//@Disabled
public class SpitOut extends AutoSupplies{
    @Override
    public void runOpMode() {

        //  Establish all hardware
        initForAutonomous();
        //  Wait until start
        initVision();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        int path = 0;

        Recognition duck = null;
        long halfSec = 500;
        runtime.reset();
        while(runtime.milliseconds() <= halfSec){
            duck = getDuckPosition();
            if(duck != null){
                break;
            }
        }
        path = getZone(duck);
        telemetry.addLine("Path: " + path);
        telemetry.update();

        //sleep(300);
        if (path == 1) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            setArmLevel(4);
            setArmEncoderMode();
            pause(2000);
        } else if (path == 2) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            setArmLevel(4);
            setArmEncoderMode();
            pause(2000);
        } else if(path == 3){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            setArmLevel(4);
            setArmEncoderMode();
            pause(2000);
        } else{ lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED); }

        encoderMove(700,0.5, 0.5);
        pause(1000);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
        setArmLevel(1);



    }
}
