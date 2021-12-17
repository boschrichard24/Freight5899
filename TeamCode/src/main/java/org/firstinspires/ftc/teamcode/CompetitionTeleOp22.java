package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="CompetitionTeleOp22", group="Linear Opmode")

public class CompetitionTeleOp22 extends AutoSupplies {

    private double leftMoveInput = 0.0;
    private double rightMoveInput = 0.0;

    private double pivotPower = 0.0;
    private double mainArmPower = 0.0;
    private double innerAngleArmPower = 0.0;

    private int currentArmLevel = 0;

    final private double clawIncrement = 0.03;
    final private double clawAngleMax = 135.0;
    final private double clawAngleMin = 0.0;

    final private double duckyPower = 0.7;

    final private double armMotorPower = 0.25;

// test
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        setup();
        resetArmEncoders();

        while (opModeIsActive()){
            leftMoveInput = gamepad1.left_stick_y;
            rightMoveInput = gamepad1.right_stick_y;

            leftMoveInput = Range.clip(leftMoveInput, -1.0, 1.0);
            rightMoveInput = Range.clip(rightMoveInput, -1.0, 1.0);

            if (gamepad1.right_bumper) {
                pivotPower = 0.5;
            }
            else if (gamepad1.left_bumper) {
                pivotPower = -0.5;
            }
            else {
                pivotPower = 0.0;
            }

            if (gamepad2.y) {
                mainArmPower = 0.5;
            }
            else if (gamepad2.b) {
                mainArmPower = -0.5;
            }
            else {
                mainArmPower = 0.0;
            }

            if (gamepad2.x) {
                innerAngleArmPower = 0.5;
            }
            else if (gamepad2.a) {
                innerAngleArmPower = -0.5;
            }
            else {
                innerAngleArmPower = 0.0;
            }

            if (gamepad2.a) {
                setArmLevel(1, currentArmLevel);
            }
            if (gamepad2.y) {
                setArmLevel(5, currentArmLevel);
            }
            if (gamepad2.x && currentArmLevel < 5) {
                setArmLevel(currentArmLevel+1, currentArmLevel);
            }
            if (gamepad2.b && currentArmLevel > 1) {
                setArmLevel(currentArmLevel-1, currentArmLevel);
            }

            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                duckyMotorPower(duckyPower);
            }
            move(leftMoveInput, rightMoveInput);
            //setArmPowers(mainArmPower, innerAngleArmPower, pivotPower);
            setArmPosition(armMotorPower);
            toggleClaw(clawIncrement, clawAngleMin, clawAngleMax, gamepad1.x, 0.0);
        }
    }

    public void setup()
    {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        left_Back_Drive  = hardwareMap.get(DcMotor .class, "left_Back_Drive");
        right_Back_Drive = hardwareMap.get(DcMotor.class, "right_Back_Drive");
        left_Front_Drive  = hardwareMap.get(DcMotor.class, "left_Front_Drive");
        right_Front_Drive = hardwareMap.get(DcMotor.class, "right_Front_Drive");
        left_Arm_Motor = hardwareMap.get(DcMotor.class, "left_Arm_Motor");
        right_Arm_Motor = hardwareMap.get(DcMotor.class, "right_Arm_Motor");

        claw_Servo = hardwareMap.get(Servo .class, "claw_Servo");
        ducky = hardwareMap.get(DcMotor.class, "ducky");

        // Set the direction for each of the motors \\
        left_Back_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Back_Drive.setDirection(DcMotor.Direction.REVERSE);
        left_Front_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Front_Drive.setDirection(DcMotor.Direction.REVERSE);
        right_Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        left_Arm_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        ducky.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
