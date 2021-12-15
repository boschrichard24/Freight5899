package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="CompetitionTeleOp22", group="Linear Opmode")

public class CompetitionTeleOp22 extends AutoSupplies {

    private double leftMoveInput = 0.0;
    private double rightMoveInput = 0.0;

    private double pivotPower = 0.0;
    private double mainArmPower = 0.0;
    private double innerAngleArmPower = 0.0;

    private int currentArmLevel = 0;
    private int previousArmLevel = 0;

    final private double clawIncrement = 0.03;
    final private double clawAngleMax = 135.0;
    final private double clawAngleMin = 0.0;

    final private double duckyPower = 0.7;

// test
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        setup();

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

            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                duckyMotorPower(duckyPower);
            }
            move(leftMoveInput, rightMoveInput);
            setArmPowers(mainArmPower, innerAngleArmPower, pivotPower);
            toggleClaw(clawIncrement, clawAngleMin, clawAngleMax, gamepad1.x);
        }
    }
}
