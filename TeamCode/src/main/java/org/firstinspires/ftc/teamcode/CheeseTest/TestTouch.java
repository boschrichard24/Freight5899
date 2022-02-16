package org.firstinspires.ftc.teamcode.CheeseTest;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="TestTouch", group="CheeseTest")
public class TestTouch extends LinearOpMode {
    //All hardware
    protected DcMotor motorFwdLeft = null;
    protected DcMotor motorFwdRight = null;
    protected DcMotor motorBackLeft = null;
    protected DcMotor motorBackRight = null;
    protected Rev2mDistanceSensor distanceLeft = null;
    protected Rev2mDistanceSensor distanceRight = null;
    protected ColorSensor colorLeft = null;
    protected RevTouchSensor touchLeft = null;
    protected RevTouchSensor touchRight = null;
    protected ElapsedTime runtime = new ElapsedTime();

    //protected CRServo servo = null;

    //Variables
    private double max = 1.0;
    double maxPower;
    double powerLim = 1;
    double moveDir = -1;
    double distance1 = 0;
    double distance2 = 0;
    double color1 = 0;
    double blue = 0;
    double green = 0;
    double red = 0;
    //double servoPower = 5;

    public void pause(long millis){
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis){

        }
    }

    @Override
    public void runOpMode() {
        //Prepares all the hardware
        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorFwdLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        distanceLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeft");
        distanceRight = hardwareMap.get(Rev2mDistanceSensor.class, "distanceRight");
        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        touchLeft = hardwareMap.get(RevTouchSensor.class, "touchLeft");
        touchRight = hardwareMap.get(RevTouchSensor.class, "touchRight");

        //servo = hardwareMap.get(CRServo.class, "servo");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //variables
        boolean changed3 = false;
        boolean changed4 = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            pause(1000);
            //L
            motorBackLeft.setPower(0.25);
            motorBackRight.setPower(0.25);
            motorFwdLeft.setPower(0.25);
            motorFwdRight.setPower(0.25);

            if (touchLeft.isPressed() || touchRight.isPressed()){
                //  Turn all motors off and sleep
                pause(5000);
                motorBackLeft.setPower(-0.25);
                motorBackRight.setPower(-0.25);
                motorFwdLeft.setPower(-0.25);
                motorFwdRight.setPower(-0.25);
            }
            else if (distanceLeft.getDistance(DistanceUnit.MM) < 20 || distanceRight.getDistance(DistanceUnit.MM) < 20){
                pause(5000);
                motorBackLeft.setPower(0.25);
                motorBackRight.setPower(0.25);
                motorFwdLeft.setPower(0.25);
                motorFwdRight.setPower(0.25);
            }
            else if(!(touchLeft.isPressed() || touchRight.isPressed())){
                idle();
            }
            else if (distanceLeft.getDistance(DistanceUnit.MM) >= 20 || distanceRight.getDistance(DistanceUnit.MM) >= 20){
                idle();
            }

            telemetry.addData("TouchLeft", touchLeft.isPressed());
            telemetry.addData("TouchRight", touchRight.isPressed());
            telemetry.update();

        }
    }


    }