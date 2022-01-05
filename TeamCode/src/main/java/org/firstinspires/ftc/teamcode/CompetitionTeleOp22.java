package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="CompetitionTeleOp22", group="Linear Opmode")

public class CompetitionTeleOp22 extends LinearOpMode {

    private double leftMoveInput      = 0.0;
    private double rightMoveInput     = 0.0;
    private double pivotPower         = 0.0;
    private double mainArmPower       = 0.0;
    private double innerAngleArmPower = 0.0;

    private int currentArmLevel       = 0;

    final private double clawIncrement = 0.03;
    final private double clawAngleMax  = 135.0;
    final private double clawAngleMin  = 0.0;
    final private double duckyPower    = 0.7;
    final private double armMotorPower = 0.25;

    // Motors, servos, etc.
    protected DcMotor left_Back_Drive = null;
    protected DcMotor right_Back_Drive = null;
    protected DcMotor left_Front_Drive = null;
    protected DcMotor right_Front_Drive = null;

    protected DcMotor left_Arm_Motor = null;
    protected DcMotor right_Arm_Motor = null;
    protected DcMotor pivot_Arm_Motor = null;

    protected Servo claw_Servo = null;  // This is the open and close servo of the claw \\
    protected DcMotor ducky = null;

    public void resetArmEncoders() {
        // Reset all encoders and set their modes \\
        left_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //left_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_Arm_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //right_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_Arm_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //pivot_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot_Arm_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setArmEncoderMode() {
        // Set the mode of the encoders to RUN_TO_POSITION \\
        left_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setArmPosition(double power) {
        // "While both motors are busy reaching their encoder targets..." \\
        while (right_Arm_Motor.isBusy() || left_Arm_Motor.isBusy()) {
            left_Arm_Motor.setPower(power);
            right_Arm_Motor.setPower(power);
        }
    }

    public void setArmLevel(int targetLevel, int lastLevel)
    {
        if (left_Arm_Motor.isBusy() || right_Arm_Motor.isBusy()) {

            //resetArmEncoders();  <==  Do this in setup     \\
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
                default:
                    encoderTargets[0] = 1000;
                    encoderTargets[1] = 0; // Default is bottom (level 1) \\
                    break;
            }
            if (targetLevel > lastLevel) {
                // "Don't change the values" \\
            } else if (targetLevel < lastLevel) {
                encoderTargets[0] *= -1;
                encoderTargets[0] *= -1;
            } else {
                encoderTargets[0] = 0;
                encoderTargets[1] = 0; // The levels are the same as previous but the function was called \\
            }

            // Reset the previous level that the robot was at and set the target encoder values \\
            lastLevel = targetLevel;

            left_Arm_Motor.setTargetPosition(encoderTargets[0]);
            right_Arm_Motor.setTargetPosition(encoderTargets[1]);
        }
    }

    public void toggleClaw(double increment, double minAngle, double maxAngle, boolean open, double newPosition)
    {
        if (open && newPosition < maxAngle) {
            newPosition += increment;
        }
        else if (!open && newPosition > minAngle) {
            newPosition -= increment;
        }
        claw_Servo.setPosition(newPosition);
    }

    public void move(double leftPower, double rightPower)
    {
        //sets the power of the motors
        left_Back_Drive.setPower(leftPower);
        left_Front_Drive.setPower(leftPower);
        right_Back_Drive.setPower(rightPower);
        right_Front_Drive.setPower(rightPower);
    }

    public void duckyMotorPower(double power)
    {
        ducky.setPower(power);
    }

    // ******************               test               ******************  \\
    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        left_Back_Drive = hardwareMap.get(DcMotor.class, "left_Back_Drive");
        left_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Back_Drive = hardwareMap.get(DcMotor.class, "right_Back_Drive");
        left_Front_Drive = hardwareMap.get(DcMotor.class, "left_Front_Drive");
        right_Front_Drive = hardwareMap.get(DcMotor.class, "right_Front_Drive");
        left_Arm_Motor = hardwareMap.get(DcMotor.class, "left_Arm_Motor");
        right_Arm_Motor = hardwareMap.get(DcMotor.class, "right_Arm_Motor");
        ducky = hardwareMap.get(DcMotor.class, "ducky");

        claw_Servo = hardwareMap.get(Servo.class, "claw_Servo");

        //left_Arm_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //right_Arm_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //pivot_Arm_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the direction for each of the motors \\
        left_Back_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Back_Drive.setDirection(DcMotor.Direction.REVERSE);
        left_Front_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Front_Drive.setDirection(DcMotor.Direction.REVERSE);
        right_Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        left_Arm_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        ducky.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        //runtime.reset();      not necessary

        //setup();
        //resetArmEncoders();

        while (opModeIsActive()) {
            rightMoveInput = gamepad1.left_stick_y;
            leftMoveInput = gamepad1.right_stick_y;

            rightMoveInput = Range.clip(leftMoveInput, -1.0, 1.0);
            leftMoveInput = Range.clip(rightMoveInput, -1.0, 1.0);

            if (gamepad1.right_bumper) {
                pivotPower = 0.5;
            } else if (gamepad1.left_bumper) {
                pivotPower = -0.5;
            } else {
                pivotPower = 0.0;
            }

            if (gamepad2.y) {
                mainArmPower = 0.5;
            } else if (gamepad2.b) {
                mainArmPower = -0.5;
            } else {
                mainArmPower = 0.0;
            }

            if (gamepad2.x) {
                innerAngleArmPower = 0.5;
            } else if (gamepad2.a) {
                innerAngleArmPower = -0.5;
            } else {
                innerAngleArmPower = 0.0;
            }

            if (gamepad2.a) {
                setArmLevel(1, currentArmLevel);
            }
            if (gamepad2.y) {
                setArmLevel(5, currentArmLevel);
            }
            if (gamepad2.x && currentArmLevel < 5) {
                setArmLevel(currentArmLevel + 1, currentArmLevel);
            }
            if (gamepad2.b && currentArmLevel > 1) {
                setArmLevel(currentArmLevel - 1, currentArmLevel);
            }

            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                duckyMotorPower(duckyPower);
            }
            else {
                duckyMotorPower(0.0);
            }
            move(leftMoveInput, rightMoveInput);
            //setArmPowers(mainArmPower, innerAngleArmPower, pivotPower);
            setArmPosition(armMotorPower);
            toggleClaw(clawIncrement, clawAngleMin, clawAngleMax, gamepad1.x, 0.0);
        }
    }
}
