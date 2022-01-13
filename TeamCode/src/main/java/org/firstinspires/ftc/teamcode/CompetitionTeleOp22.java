package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="CompetitionTeleOp22", group="Linear Opmode")

public class CompetitionTeleOp22 extends LinearOpMode {


    // ******************               VARIABLE DEF-S              ******************  \\

    // Power vars
    private double leftMovePower  = 0.0;
    private double rightMovePower = 0.0;
    private double pivotPower     = 0.0;
    private double duckyPower     = 0.2;
    // Misc. vars
    private double spin         = 0.0;
    private ElapsedTime runtime = new ElapsedTime();
    private int level = 1;
    public RevBlinkinLedDriver lights;
    // Claw vars
    protected Servo claw        = null;  // This is the open and close servo of the claw \\
    final private double clawMax       = 0.401;
    final private double clawMin       = 0.601;
    private double clawPos             = 0.0;
    private double powerChange         = 1.0;
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
    boolean changed1 = false;
    boolean changed2 = false;
    boolean changed3 = false;
    boolean changed4 = false;
    boolean changed5 = false;
    boolean changed6 = false;
    boolean changed7 = false;

    boolean changedToNewLevel = false;


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

    public void setArmLevel(int targetLevel)
    {
        int[] encoderTargets = new int[2];

        switch (targetLevel) {
            case 1:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                encoderTargets[0] = 0;
                encoderTargets[1] = 0; // floor level to pick up pieces \\
                break;
            case 2:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                encoderTargets[0] = 0;
                encoderTargets[1] = 100; // level 1 on shipping container \\
                break;
            case 3:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                encoderTargets[0] = 0;
                encoderTargets[1] = 130; // level 2 on shipping container \\
                break;
            case 4:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                //put light code here plug in light into the BLinkin - Blikin plugs inot the servo port - config inside of servo
                encoderTargets[0] = 0;
                encoderTargets[1] = 300; // level 3 on shipping container \\
                break;
            case 5:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
                encoderTargets[0] = 0;
                encoderTargets[1] = 500; // top of shipping container for gamepiece \\
                break;
            case 6:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                encoderTargets[0] = 0;
                encoderTargets[1] = 530; // high as possible (Caed.. we need this?? :\ ) \\
                break;
            case 7:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                //whatever postiion you want for init
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void move()
    {
        //  S P E E D   \\
        leftMovePower = -gamepad1.left_stick_y * powerChange;
        rightMovePower = -gamepad1.right_stick_y * powerChange;
        left_Back_Drive.setPower(leftMovePower);
        right_Back_Drive.setPower(rightMovePower);
        left_Front_Drive.setPower(leftMovePower);
        right_Front_Drive.setPower(rightMovePower);

        if(gamepad1.a && !changed1) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
            if(powerChange == 0.5 || powerChange == 2) {
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
            if(powerChange == 0.5 || powerChange == 2) {
                powerChange = 1;
            }
            else {
                powerChange = 2;
            }
            changed7 = true;
        }
        else if (!gamepad1.b){
            changed7 = false;
        }
        //   I N V E R S E   \\
        if(gamepad1.x && !changed2) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE);
            leftMovePower = gamepad1.left_stick_y;
            rightMovePower = gamepad1.right_stick_y;
            changed2 = true;
        }
        else if(!gamepad1.x){
            changed2 = false;
        }
    }

    public void pivotArmPower()
    {
        if(gamepad1.left_trigger>0.1 && !(gamepad1.right_trigger>0.1)) {
            spin = gamepad1.left_trigger*0.5;
            pivot_Arm_Motor.setPower(spin);
            //pivotPower = Range.clip(spin, 0, 0.5);
            //pivot_Arm_Motor.setPower(pivotPower);
        }
        else if(gamepad1.right_trigger>0.1 && !(gamepad1.left_trigger>0.1)){
            spin = -gamepad1.right_trigger*0.5;
            //pivotPower = Range.clip(spin, 0, 0.5);
            //pivot_Arm_Motor.setPower(pivotPower);
            pivot_Arm_Motor.setPower(spin);
        }
    }

    public void armFunction()
    {
        int prevLevel = level;

        // "if you press up and you're not at the needed higher level..." \\
        if (gamepad2.dpad_up && !changedToNewLevel) {
            if (level >= 6) { level = 6; }
            else if (level < 6) { level ++; }
            changedToNewLevel = true;
        }
        // "if you press down and you're not at the needed lower level..." \\
        else if (gamepad2.dpad_down && !changedToNewLevel) {
            if (level <= 1) { level = 1; }
            else if (level > 1) { level --; }
            changedToNewLevel = true;
        }
        else if (gamepad2.a && !changedToNewLevel) {
            level = 1;
            changedToNewLevel = true;
        }
        // "if neither buttons pressed, reset the button toggle bool" (allows you to press again) \\
        else if (!gamepad2.dpad_up && !gamepad2.dpad_down) { changedToNewLevel = false; }

        // This will check if a new level if asked for and change the encoder modes \\
        // "if the level from one frame ago was not the same... " \\
        if (level != prevLevel) {
            right_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setArmLevel(level);
            prevLevel = level;
        }
        // "if the level has not changed, turn off motor and motor encoders" \\
        else {
            right_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            left_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runArmPower(0.0);
            prevLevel = level;
        }
    }


    @Override
    public void runOpMode() {

        waitForStart();

//  E N C O D E R S SET up  \\
        resetArmEncoders();

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);

        while (opModeIsActive()) {
//  M O V E  func  \\
            move();

//  S P I N  func  \\
            pivotArmPower();

//  D U C K Y func  \\
            runtime.reset();
            if(gamepad1.right_bumper && !ducky.isBusy()){
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE);
                long millis = 2000;
                runtime.reset();
                while(runtime.milliseconds() <= millis){
                    ducky.setPower(duckyPower);
                    duckyPower += 0.001;
                }
                ducky.setPower(0);
            }

//  A R M  func  \\
            /*if(gamepad2.dpad_up && !changed3){
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
                level = 3;
                changed6 = true;
            }else if(!gamepad2.y){changed6 = false;} */
            // This will set a new level when you press the dpad button up and \\
            // will wait until you let go to be able to go to a new level \\
            armFunction();

//  C L A W func \\
            if(gamepad2.x && clawPos < clawMax){
                while(clawPos < clawMax) {
                    clawPos += 0.3;
                }
                claw.setPosition(clawPos);
            }
            else if(gamepad2.b && clawPos > clawMin){
                while(clawPos > clawMin) {
                    clawPos -= 0.3;
                }
                claw.setPosition(clawPos);
            }

            // ******************               OVERIDE ARM func              ******************  \\

            if(gamepad2.left_trigger>0.5 && gamepad2.right_trigger>0.5){
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
                right_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                left_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                double armLeftPower;
                double armRightPower;
                double armDriveLeft  = -gamepad2.left_stick_y;
                double armDriveRight = -gamepad2.right_stick_y;
                armLeftPower  = Range.clip(armDriveLeft, -0.5, 0.5);
                armRightPower = Range.clip(armDriveRight, -0.5, 0.5);
                left_Arm_Motor.setPower(armLeftPower);
                right_Arm_Motor.setPower(armRightPower);
            }

//  T E M E T R Y  D A T A  \\
            telemetry.addData("ArmLevel : ", level);
            telemetry.addData("Swivel Left Power:", left_Arm_Motor.getPower());
            telemetry.addData("Swivel Right Power:", right_Arm_Motor.getPower());

            //get updated encoder positions
            telemetry.addData("Arm Left Value:",  left_Arm_Motor.getCurrentPosition());
            telemetry.update();
            telemetry.addData("Arm Right Value:",  right_Arm_Motor.getCurrentPosition());
            telemetry.update();
        }
    }
}