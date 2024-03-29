package org.firstinspires.ftc.teamcode.TeleOpPrograms22;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="LevelsTestOp", group="Linear Opmode")
public class LevelsTeleOp extends LinearOpMode{


    // ******************               VARIABLE DEF-S              ******************  \\

    // Power vars
    private double leftMovePower        = 0.0;
    private double rightMovePower       = 0.0;
    //private double pivotPower         = 0.0;
    private double duckyPower           = 0.57;
    // Misc. vars
    private ElapsedTime runtime = new ElapsedTime();
    private int level                   = 4;
    private double powerChange          = 1.0;
    protected RevBlinkinLedDriver lights;
    // Claw vars
    protected CRServo basket            = null;  // This is the Continuous servo for the intake basket \\
    private double basketPower          = 5.0;
    // Motors vars
    protected DcMotor left_Back_Drive   = null;
    protected DcMotor right_Back_Drive  = null;
    protected DcMotor left_Front_Drive  = null;
    protected DcMotor right_Front_Drive = null;
    protected DcMotor left_Arm_Motor    = null;
    protected DcMotor right_Arm_Motor   = null;
    protected DcMotor pivot_Arm_Motor   = null;
    protected DcMotor ducky             = null;  // Motor on the back of the robot to spin the carousel wheel \\
    // Button Booleans
    private boolean changed1            = false;
    private boolean changed2            = false;
    private boolean changed3            = false;
    private boolean changed4            = false;
    private boolean changed5            = false;
    private boolean changed6            = false;
    private boolean changed7            = false;
    private boolean changed1Zero        = false;
    private boolean changed2Zero        = false;
    private boolean lower1changed       = false;
    private boolean lower2changed       = false;
    //Sensor Variables
    protected RevTouchSensor touchLeft  = null;
    protected RevTouchSensor touchRight = null;

//    **********     MAIN FUNCTIONS     **********     \\


    public void resetArmEncoders()
    {
        left_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // you need to change the setMode to RUN_TO_POSITION when you want to change level \\
    }

    public void pause(long millis){
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis){

        }
    }

    public void runArmPower(double power)
    {
        left_Arm_Motor.setPower(power);
        right_Arm_Motor.setPower(power);
    }

    public void setArmLevel(int targetLevel) {
        int[] encoderTargets = new int[2];

        switch (targetLevel) {
            case 1:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                encoderTargets[0] = -800;
                encoderTargets[1] = -950;
                break;
            case 2:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                encoderTargets[0] = -900;
                encoderTargets[1] = -1000; // level 1 on shipping container \\
                break;
            case 3:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                encoderTargets[0] = -810;
                encoderTargets[1] = -935; // level 2 on shipping container \\
                break;
            case 4:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                encoderTargets[0] = -720;
                encoderTargets[1] = -700; // level 3 on shipping container \\
                break;
            case 5:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
                encoderTargets[0] = -200;
                encoderTargets[1] = -600; // top of shipping container for gamepiece \\
                break;
            case 6:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                encoderTargets[0] = 32;
                encoderTargets[1] = -462; // high as possible (Caed.. we need this?? :\ ) \\
                break;
            case 7:
                break;


            default:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY);
                encoderTargets[0] = 0;
                encoderTargets[1] = 0; // Default is bottom (level 1) \\
                break;
        }
        //if(targetLevel != 1 || targetLevel != 7){
        runArmPower(.35);
        left_Arm_Motor.setTargetPosition(encoderTargets[0]);
        right_Arm_Motor.setTargetPosition(encoderTargets[1]);
        //}
        /*else if(targetLevel == 1 || targetLevel == 7){
            runArmPower(0);
            resetArmEncoders();
            left_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            encoderTargets[0] = 0;
            encoderTargets[1] = 0;
            targetLevel = 7;
        }*/


    }

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
        touchLeft = hardwareMap.get(RevTouchSensor.class, "touchLeft");
        touchRight = hardwareMap.get(RevTouchSensor.class, "touchRight");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        pivot_Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//  Set the direction for each of the motors  \\
        left_Back_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Back_Drive.setDirection(DcMotor.Direction.REVERSE);
        left_Front_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Front_Drive.setDirection(DcMotor.Direction.REVERSE);
        right_Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        left_Arm_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot_Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        ducky.setDirection(DcMotorSimple.Direction.FORWARD);

        basket.setDirection(CRServo.Direction.FORWARD);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }



    @Override
    public void runOpMode() {

        hardwareSetup();

        waitForStart();

//  E N C O D E R S SET up  \\
        resetArmEncoders();
        left_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (opModeIsActive()) {

            leftMovePower = -gamepad1.left_stick_y*powerChange;
            rightMovePower = -gamepad1.right_stick_y*powerChange;
            left_Back_Drive.setPower(leftMovePower);
            right_Back_Drive.setPower(rightMovePower);
            left_Front_Drive.setPower(leftMovePower);
            right_Front_Drive.setPower(rightMovePower);

            if(gamepad1.a && !changed1) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
                if(powerChange == 0.5 || powerChange == 3) {
                    powerChange = 1;
                }
                else {
                    powerChange = 0.5;
                }
                changed1 = true;
            }
            else if (!gamepad1.a){
                changed1 = false;
            }

            if(gamepad1.b && !changed7) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
                if(powerChange == 0.5 || powerChange == 3) {
                    powerChange = 1;
                }
                else {
                    powerChange = 3;
                }
                changed7 = true;
            }
            else if (!gamepad1.b){
                changed7 = false;
            }

            //   I N V E R S E   \\
            if(gamepad1.x && !changed2) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE);
                if(powerChange > 0) {
                    powerChange = -1;
                }
                else{
                    powerChange = 1;
                }
                changed2 = true;
            }
            else if(!gamepad1.x){
                changed2 = false;
            }

//  S P I N  func  \\
            if(gamepad2.left_bumper && !(gamepad2.right_bumper)) {
                pivot_Arm_Motor.setPower(-0.65);
                //pivotPower = Range.clip(spin, 0, 0.5);
                //pivot_Arm_Motor.setPower(pivotPower);
            }
            else if(gamepad2.right_bumper && !(gamepad2.left_bumper)){
                //pivotPower = Range.clip(spin, 0, 0.5);
                //pivot_Arm_Motor.setPower(pivotPower);
                pivot_Arm_Motor.setPower(0.65);
            }
            else{
                pivot_Arm_Motor.setPower(0);
            }
//  D U C K Y func  \\
            if (gamepad1.right_bumper && !ducky.isBusy()) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
                long millis = 1100;
                runtime.reset();
                while(runtime.milliseconds() <= millis){
                    ducky.setPower(duckyPower);
                }
                millis = 600;
                runtime.reset();
                while(runtime.milliseconds() <= millis){
                    ducky.setPower(duckyPower+1.5);
                }
                ducky.setPower(0);
            }
            else if (gamepad1.left_bumper && !ducky.isBusy()) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
                long millis = 1100;
                runtime.reset();
                while (runtime.milliseconds() <= millis) {
                    ducky.setPower(-duckyPower);
                }
                millis = 600;
                runtime.reset();
                while (runtime.milliseconds() <= millis) {
                    ducky.setPower(-duckyPower - 1.5);
                }
                ducky.setPower(0);
            }

//  A R M  func  \\
            if(gamepad2.dpad_up && !changed3){
                if(level == 7){level = 0;}
                level ++;
                if(level > 6){level = 6;}
                changed3 = true;
            }else if(!gamepad2.dpad_up){changed3 = false;}
            if(gamepad2.dpad_down && !changed4){
                if(level == 7){level = 2;}
                level --;
                if(level < 3){level = 3;}
                changed4 = true;
            }else if(!gamepad2.dpad_down){changed4 = false;}
            if(gamepad2.x && !changed5){
                level = 3;
                changed5 = true;
            }else if(!gamepad2.x){changed5 = false;}
            if(gamepad2.y && !changed6){
                level = 6;
                changed6 = true;
            }else if(!gamepad2.y){changed6 = false;}
            if(gamepad2.dpad_left && !lower1changed){
                level = 2;
            } else if(!gamepad2.dpad_left) { lower1changed = false; }
            if(gamepad2.dpad_right && !lower2changed){
                level = 1;
            } else if(!gamepad2.dpad_right){ lower2changed = false; }

            setArmLevel(level);

            if(gamepad2.left_stick_button && !changed1Zero){
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
                left_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                right_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                runArmPower(0);
            } else if(!gamepad2.left_stick_button) {
                resetArmEncoders();
                changed1Zero = false;
                left_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }



//  I N T A K E func \\
            if(gamepad2.a && !gamepad2.b) {
                basket.setPower(basketPower);
            }
            else if(gamepad2.b && !gamepad2.a) {
                basket.setPower(-basketPower);
            }
            else {
                basket.setPower(0.0);
            }

//  T E M E T R Y  D A T A  \\
            telemetry.addData("  <===============>  ", "");

            telemetry.addData("Current Arm Level: ", level);
            telemetry.addData("Arm Left Encoder Value: ",  left_Arm_Motor.getCurrentPosition());
            telemetry.addData("Arm Right Encoder Value: ",  right_Arm_Motor.getCurrentPosition());

            telemetry.addData("  ----------  ", "");

            telemetry.addData("Button/Input X is on: ", gamepad2.x);
            telemetry.addData("Button/Input Y is on: ", gamepad2.y);
            telemetry.addData("Button/Input B is on: ", gamepad2.b);
            telemetry.addData("Button/Input A is on: ", gamepad2.a);
            telemetry.addData("TouchLeft: ", touchLeft.isPressed());
            telemetry.addData("TouchRight: ", touchRight.isPressed());


            telemetry.addData("  <===============>  ", "");
            telemetry.update();
        }
    }
}
