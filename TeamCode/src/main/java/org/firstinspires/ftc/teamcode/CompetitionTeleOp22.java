package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="CompetitionTeleOp22", group="Linear Opmode")

public class CompetitionTeleOp22 extends LinearOpMode {


    // ******************               VARIABLE DEFS              ******************  \\


    // Power vars
    private double leftMovePower  = 0.0;
    private double rightMovePower = 0.0;
    private double pivotPower     = 0.0;
    private double duckyPower     = 0.7;
    // Misc. vars
    private double spin         = 0.0;
    private ElapsedTime runtime = new ElapsedTime();
    private int level = 0;
    // Claw vars
    protected Servo claw        = null;  // This is the open and close servo of the claw \\
    final private double clawMax       = 135.0;
    final private double clawMin       = 0.0;
    private double clawPos            = 0.0;
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



//    **********     MISC FUNCTIONS     **********     \\


    public void resetArmEncoders()
    {
        left_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setRunEncoders()
    {
        left_Arm_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_Arm_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                encoderTargets[0] = 1000;
                encoderTargets[1] = 0; // floor level to pick up pieces \\
                break;
            case 2:
                encoderTargets[0] = 1100;
                encoderTargets[1] = 0; // level 1 on shipping container \\
                break;
            case 3:
                encoderTargets[0] = 1100;
                encoderTargets[1] = 400; // level 2 on shipping container \\
                break;
            case 4:
                //put light code here plug in light into the BLinkin - Blikin plugs inot the servo port - config inside of servo
                encoderTargets[0] = 1000;
                encoderTargets[1] = 800; // level 3 on shipping container \\
                break;
            case 5:
                encoderTargets[0] = 1300;
                encoderTargets[1] = 1000; // top of shipping container for gamepiece \\
                break;
            case 6:
                encoderTargets[0] = 1500;
                encoderTargets[1] = 1000; // high as possible (Caed.. we need this?? :\ ) \\
                break;
            case 7:
                //whatever postiion you want for init
            default:
                encoderTargets[0] = 1000;
                encoderTargets[1] = 0; // Default is bottom (level 1) \\
                break;
        }
        runArmPower(.5);
        left_Arm_Motor.setTargetPosition(encoderTargets[0]);
        right_Arm_Motor.setTargetPosition(encoderTargets[1]);
    }


    // ******************               test func              ******************  \\


    @Override
    public void runOpMode() {
//  Connect Motors to Phone  \\
        left_Back_Drive = hardwareMap.get(DcMotor.class, "left_Back_Drive");
        right_Back_Drive = hardwareMap.get(DcMotor.class, "right_Back_Drive");
        left_Front_Drive = hardwareMap.get(DcMotor.class, "left_Front_Drive");
        right_Front_Drive = hardwareMap.get(DcMotor.class, "right_Front_Drive");
        left_Arm_Motor = hardwareMap.get(DcMotor.class, "left_Arm_Motor");
        right_Arm_Motor = hardwareMap.get(DcMotor.class, "right_Arm_Motor");
        pivot_Arm_Motor = hardwareMap.get(DcMotor.class, "pivot_Arm_Motor");
        ducky = hardwareMap.get(DcMotor.class, "ducky");

        claw = hardwareMap.get(Servo.class, "claw_Servo");

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

        waitForStart();
        //runtime.reset();      not necessary

//  E N C O D E R S SET up  \\
        resetArmEncoders();
        left_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //setRunEncoders();

        while (opModeIsActive()) {
//  M O V E  func  \\
            //  S P E E D   \\
            if(gamepad1.a && !changed1){

            }
            leftMovePower = -gamepad1.left_stick_y;
            rightMovePower = -gamepad1.right_stick_y;
            left_Back_Drive.setPower(leftMovePower);
            right_Back_Drive.setPower(rightMovePower);
            left_Front_Drive.setPower(leftMovePower);
            right_Front_Drive.setPower(rightMovePower);
            if(gamepad1.a && !changed1){
                double lfSpeed = Range.clip(leftMovePower, -5, 5);
                double rtSpeed = Range.clip(rightMovePower, -5, 5);
            }
            changed1 = true;
            if(!gamepad1.a && changed1){
                changed1 = false;
            }

            //   I N V E R S E   \\
            if(gamepad1.x && !changed2){
                leftMovePower = gamepad1.left_stick_y;
                rightMovePower = gamepad1.right_stick_y;
            }
            changed2 = true;
            if(!gamepad1.x && changed2){
                changed2 = false;
            }

//  S P I N  func  \\
            if(gamepad1.left_trigger>0 && !(gamepad1.right_trigger>0)) {
                spin = gamepad1.left_trigger*0.5;
                pivot_Arm_Motor.setPower(spin);
                //pivotPower = Range.clip(spin, 0, 0.5);
                //pivot_Arm_Motor.setPower(pivotPower);
            }
            else if(gamepad1.right_trigger>0 && !(gamepad1.left_trigger>0)){
                spin = -gamepad1.right_trigger;
                //pivotPower = Range.clip(spin, 0, 0.5);
                //pivot_Arm_Motor.setPower(pivotPower);
                pivot_Arm_Motor.setPower(spin);
            }

//  D U C K Y func  \\
            if(gamepad1.left_bumper && !ducky.isBusy()){
                for(int i=0; i<50000; i++){            // NEED A GOOD WAY TO DELAY INCRE BY 1 SEC
                    ducky.setPower(duckyPower);
                    duckyPower *= 1.1;
                }
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
                level = 3;
                changed6 = true;
            }else if(!gamepad2.y){changed6 = false;}

            setArmLevel(level);
//  C L A W func \\
            if(gamepad2.dpad_up && clawPos < clawMax){
                while(clawPos < clawMax) {
                    clawPos += 0.03;
                }
            }
            else if(gamepad2.dpad_down && clawPos > clawMin){
                while(clawPos > clawMin) {
                    clawPos -= 0.03;
                }
            }


            // ******************               OVERIDE ARM func              ******************  \\


            if(gamepad2.left_trigger>0.5 && gamepad2.right_trigger>0.5){
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
            telemetry.addData("Current Arm Level:", level);

            telemetry.addData("Swivel Left Power:", left_Arm_Motor.getPower());
            telemetry.addData("Swivel Right Power:", right_Arm_Motor.getPower());

            //get updated encoder positions
            telemetry.addData("Arm Left Value:",  left_Arm_Motor.getCurrentPosition());
            telemetry.addData("Arm Right Value:",  right_Arm_Motor.getCurrentPosition());
        }
    }
}

//if (gamepad2.a && (!left_Arm_Motor.isBusy() && !right_Arm_Motor.isBusy())) {
//                setArmLevel(1);
//                runArmToPosition(0.5);
//            }
//            else if (gamepad2.x && (!left_Arm_Motor.isBusy() && !right_Arm_Motor.isBusy())) {
//                setArmLevel(2);
//                runArmToPosition(0.5);
//            }
//            else if (gamepad2.y && (!left_Arm_Motor.isBusy() && !right_Arm_Motor.isBusy())) {
//                setArmLevel(3);
//                runArmToPosition(0.5);
//            }
//            else if (gamepad2.b && (!left_Arm_Motor.isBusy() && !right_Arm_Motor.isBusy())) {
//                setArmLevel(4);
//                runArmToPosition(0.5);
//            }
//            else if (gamepad2.right_bumper && (!left_Arm_Motor.isBusy() && !right_Arm_Motor.isBusy())) {
//                setArmLevel(5);
//                runArmToPosition(0.5);
//            }
//            else if (gamepad2.left_bumper && (!left_Arm_Motor.isBusy() && !right_Arm_Motor.isBusy())) {
//                setArmLevel(6);
//                runArmToPosition(0.5);
//            }