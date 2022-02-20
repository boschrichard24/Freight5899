package org.firstinspires.ftc.teamcode.TeleOpPrograms22;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Light Show", group="Linear Opmode")

public class LightShow extends LinearOpMode {

    protected DcMotor left_Back_Drive   = null;
    protected DcMotor right_Back_Drive  = null;
    protected DcMotor left_Front_Drive  = null;
    protected DcMotor right_Front_Drive = null;
    protected DcMotor left_Arm_Motor    = null;
    protected DcMotor right_Arm_Motor   = null;
    protected DcMotor pivot_Arm_Motor   = null;
    protected DcMotor ducky             = null;

    public RevBlinkinLedDriver lights;
    protected CRServo basket        = null;
    private ElapsedTime runtime = new ElapsedTime();


    public void hardwareSetup()
    {
        //  Connect Motors to Phone  \\
        left_Back_Drive = hardwareMap.get(DcMotor.class, "left_Back_Drive");
        right_Back_Drive = hardwareMap.get(DcMotor.class, "right_Back_Drive");
        left_Front_Drive = hardwareMap.get(DcMotor.class, "left_Front_Drive");
        right_Front_Drive = hardwareMap.get(DcMotor.class, "right_Front_Drive");
        left_Arm_Motor = hardwareMap.get(DcMotor.class, "left_Arm_Motor");
        right_Arm_Motor = hardwareMap.get(DcMotor.class, "right_Arm_Motor");
        pivot_Arm_Motor = hardwareMap.get(DcMotor.class, "pivot_Arm_Motor");
        ducky = hardwareMap.get(DcMotor.class, "ducky");

        basket = hardwareMap.get(CRServo.class, "basket");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

//  Set the direction for each of the motors  \\
        left_Back_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Back_Drive.setDirection(DcMotor.Direction.REVERSE);
        left_Front_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Front_Drive.setDirection(DcMotor.Direction.REVERSE);
        right_Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        left_Arm_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot_Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        ducky.setDirection(DcMotorSimple.Direction.FORWARD);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    //  Pause for the specified amount of time (time: mili secs)
    public void pause(long millis){
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis){

        }
    }

    @Override
    public void runOpMode() {

        hardwareSetup();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
        waitForStart();

        while (opModeIsActive()) {
            ducky.setPower(0.08);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
            pause(22000);

            runtime.reset();
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
            pause(25000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
            pause(25000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
            pause(25000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
            pause(25000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE);
            pause(25000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
            pause(25000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
            pause(25000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE);
            pause(25000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE);
            pause(25000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
            pause(25000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
            pause(25000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
            pause(25000);

            if(runtime.seconds() > 280){
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
                pause(12000);
            }
            runtime.reset();

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
            pause(30000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
            pause(30000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE);
            pause(30000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
            pause(30000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);
            pause(30000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
            pause(30000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
            pause(30000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
            pause(30000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
            pause(30000);

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
            pause(30000);

            if(runtime.seconds() > 280){
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                pause(20000);
            }

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);

        }


    }
}
