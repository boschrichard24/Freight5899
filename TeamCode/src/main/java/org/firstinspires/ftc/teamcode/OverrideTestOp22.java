package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="OverrideTestOp22", group="Linear Opmode")
public class OverrideTestOp22 extends LinearOpMode{


    // ******************               VARIABLE DEF-S              ******************  \\

    // Power vars
    private double leftMovePower  = 0.0;
    private double rightMovePower = 0.0;
        //private double pivotPower     = 0.0;
    private double duckyPower     = 0.57;
    // Misc. vars

    private ElapsedTime runtime = new ElapsedTime();

    private int level = 4;
    double powerChange = 1.0;
    public RevBlinkinLedDriver lights;
    // Claw vars
    protected Servo claw        = null;  // This is the open and close servo of the claw \\
    final private double clawClosed       = 0.363;
    final private double clawOpen         = 0.611;
    private double clawPos                = 0.0;
    // Motors vars
    protected DcMotor left_Back_Drive   = null;
    protected DcMotor right_Back_Drive  = null;
    protected DcMotor left_Front_Drive  = null;
    protected DcMotor right_Front_Drive = null;
    protected DcMotor left_Arm_Motor    = null;
    protected DcMotor right_Arm_Motor   = null;
    protected DcMotor pivot_Arm_Motor   = null;
    protected DcMotor ducky             = null;
    // Button Booleans
    boolean changed1        = false;
    boolean changed2        = false;
    boolean changed3        = false;
    boolean changed4        = false;
    boolean changed5        = false;
    boolean changed6        = false;
    boolean changed7        = false;
    boolean changedOverride = false;
    boolean changedZer0     = false;

//    **********     MAIN FUNCTIONS     **********     \\


    public void resetArmEncoders()
    {
        left_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // you need to change the setMode to RUN_TO_POSITION when you want to change level \\
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
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                encoderTargets[0] = -607;
                //sleep(100);
                encoderTargets[1] = -949; // floor level to pick up pieces \\
                break;
            case 2:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                encoderTargets[0] = -432;
                encoderTargets[1] = -842; // level 1 on shipping container \\
                break;
            case 3:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                encoderTargets[0] = -300;
                encoderTargets[1] = -742; // level 2 on shipping container \\
                break;
            case 4:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                encoderTargets[0] = -181;
                encoderTargets[1] = -609; // level 3 on shipping container \\
                break;
            case 5:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
                encoderTargets[0] = -12;
                encoderTargets[1] = -587; // top of shipping container for gamepiece \\
                break;
            case 6:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                encoderTargets[0] = 76;
                encoderTargets[1] = -448; // high as possible (Caed.. we need this?? :\ ) \\
                break;
            case 7:
                // Override Path :
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE);
                encoderTargets[0] = -131;
                encoderTargets[1] = -695; // level 3 on shipping container \\
                runArmPower(.5);
                left_Arm_Motor.setTargetPosition(encoderTargets[0]);
                right_Arm_Motor.setTargetPosition(encoderTargets[1]);
                sleep(1000);
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
                encoderTargets[0] = -131;
                encoderTargets[1] = 0; // go home \\
                runArmPower(.5);
                left_Arm_Motor.setTargetPosition(encoderTargets[0]);
                right_Arm_Motor.setTargetPosition(encoderTargets[1]);
                resetArmEncoders();
                left_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                level = 5;
                encoderTargets[0] = -12;
                encoderTargets[1] = -587;
                telemetry.addData("Status", "Please Reset TeleOp");
                telemetry.update();


            default:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY);
                encoderTargets[0] = 0;
                encoderTargets[1] = 0; // Default is bottom (level 1) \\
                break;
        }
            runArmPower(.5);
            left_Arm_Motor.setTargetPosition(encoderTargets[0]);
            right_Arm_Motor.setTargetPosition(encoderTargets[1]);


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

        claw = hardwareMap.get(Servo.class, "claw");
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
                pivot_Arm_Motor.setPower(-0.35);
                //pivotPower = Range.clip(spin, 0, 0.5);
                //pivot_Arm_Motor.setPower(pivotPower);
            }
            else if(gamepad2.right_bumper && !(gamepad2.left_bumper)){
                //pivotPower = Range.clip(spin, 0, 0.5);
                //pivot_Arm_Motor.setPower(pivotPower);
                pivot_Arm_Motor.setPower(0.35);
            }
            else{
                pivot_Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                pivot_Arm_Motor.setPower(0);
            }
//  D U C K Y func  \\
            if (gamepad1.right_bumper && !ducky.isBusy()) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE);
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
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE);
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
                if(level < 1){level = 1;}
                changed4 = true;
            }else if(!gamepad2.dpad_down){changed4 = false;}
            if(gamepad2.x && !changed5){
                level = 1;
                changed5 = true;
            }else if(!gamepad2.x){changed5 = false;}
            if(gamepad2.y && !changed6){
                level = 6;
                changed6 = true;
            }else if(!gamepad2.y){changed6 = false;}
/*
            //accesses the override
            if(gamepad2.left_stick_button && !changedOverride){
                level = 7;
                changedOverride = true;
            }else if(!gamepad2.left_stick_button){changedOverride = false; level = 3;}
*/
            setArmLevel(level);

//  C L A W func \\
            if(gamepad2.a && clawPos < clawOpen){
                while(clawPos < clawOpen) {
                    clawPos += 0.05;
                }
                claw.setPosition(clawPos);

            }
            else if(gamepad2.b && clawPos > clawClosed){
                while(clawPos > clawClosed) {
                    clawPos -= 0.05;
                }
                claw.setPosition(clawPos);
            }
/*
            if(gamepad2.right_stick_button && !changedZer0){
                runArmPower(0.0);
                changedZer0 = true;
            }else if(!gamepad2.right_stick_button){changedZer0 = false; }
*/

//  T E M E T R Y  D A T A  \\
            telemetry.addData("  <===============>  ", "");

            telemetry.addData("Current Arm Level: ", level);
            telemetry.addData("Arm Left Encoder Value: ",  left_Arm_Motor.getCurrentPosition());
            telemetry.addData("Arm Right Encoder Value: ",  right_Arm_Motor.getCurrentPosition());
            //telemetry.addData("Pivot Encoder Value: ",  pivot_Arm_Motor.getCurrentPosition());

            telemetry.addData("  ----------  ", "");

            telemetry.addData("Button/Input X is on: ", gamepad2.x);
            telemetry.addData("Button/Input Y is on: ", gamepad2.y);
            telemetry.addData("Button/Input B is on: ", gamepad2.b);
            telemetry.addData("Button/Input A is on: ", gamepad2.a);

            telemetry.addData("  ----------  ", "");

            telemetry.addData("  <===============>  ", "");
            telemetry.update();
        }
    }
}

