package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="CompetitionTeleOp22", group="Linear Opmode")

public class CompetitionTeleOp22 extends AutoSupplies {

    private double leftMoveInput = 0.0;
    private double rightMoveInput = 0.0;

    private double pivotPower = 0.0;
    private double mainArmPower = 0.0;
    private double innerAngleArmPower = 0.0;

    private double clawIncrement = 0.03;
    private double clawAngleMax = 135.0;
    private double clawAngleMin = 0.0;

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
                pivotPower = 5.0;
            }
            else if (gamepad1.left_bumper) {
                pivotPower = -5.0;
            }
            else {
                pivotPower = 0.0;
            }

            if (gamepad2.y) {
                mainArmPower = 5.0;
            }
            else if (gamepad2.a) {
                mainArmPower = -5.0;
            }
            else {
                mainArmPower = 0.0;
            }

            if (gamepad2.x) {
                innerAngleArmPower = 5.0;
            }
            else if (gamepad2.b) {
                innerAngleArmPower = -5.0;
            }
            else {
                innerAngleArmPower = 0.0;
            }

            move(leftMoveInput, rightMoveInput);
            setArmPowers(mainArmPower, innerAngleArmPower, pivotPower);
            toggleClaw(clawIncrement, clawAngleMin, clawAngleMax, gamepad1.x);
        }
    }
}
