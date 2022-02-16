package org.firstinspires.ftc.teamcode.CheeseTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="TouchSensorConcept", group="CheeseTest")

public class TouchSensorConcept extends CheeseAutoSupplies {
    @Override
    public void runOpMode() {

        //  Establish all hardware
        initForAutonomous();

        //  Wait until start
        waitForStart();

        pause( 1000 );
        //L
        motorBackLeft.setPower(0.25);
        motorBackRight.setPower(0.25);
        motorFwdLeft.setPower(0.25);
        motorFwdRight.setPower(0.25);

        while(touchLeft.isPressed() && touchRight.isPressed()){
            idle();
        }
        telemetry.addData("TouchLeft", touchLeft.isPressed());
        telemetry.addData("TouchRight", touchRight.isPressed());
        telemetry.update();
            pause(5000);


        //  Turn all motors off and sleep
        motorBackLeft.setPower(-0.25);
        motorBackRight.setPower(-0.25);
        motorFwdLeft.setPower(-0.25);
        motorFwdRight.setPower(-0.25);

        while(touchLeft.isPressed() && touchRight.isPressed()){
            idle();
        }

        telemetry.addData("TouchLeft", touchLeft.isPressed());

    }
}