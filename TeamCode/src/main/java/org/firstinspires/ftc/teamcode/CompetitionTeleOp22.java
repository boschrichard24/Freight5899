package org.firstinspires.ftc.teamcode;

@TeleOp(name="CompetitionTeleOp22", group="Linear Opmode")

public class CompetitionTeleOp22 {

    AutoSupplies autoSupplies = new AutoSupplies();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            autoSupplies.test();
        }
    }
}
