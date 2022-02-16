package org.firstinspires.ftc.teamcode.CheeseTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="TouchSensorConcept", group="CheeseTest")

public class TouchSensorConcept extends AutoSupplies {
    @Override
    public void runOpMode() {

        //  Establish all hardware
        initForAutonomous();

        //  Wait until start
        waitForStart();

        pause( 1000 );
        //L
        move(3000,0,-0.3);

        if(touchLeft.isPressed() || touchRight.isPressed()){
            pause(100);
            move(500, 0, 0.3);

            pause(100);
            move(1000, 0.3, 0.3);
        }

        pause(100);
        move(1000, 1, 0);
        pause(500);
        //Transfer to O
        move(2000,.5, 0);
        pause(1000);
        //O
        move(1000,.5, .5);
        pause(100);
        move(1000,-.5, .5);
        pause(100);
        move(1000,-.5, -.5);
        pause(100);
        move(1000,.5, -.5);
        pause(100);

        //  Turn all motors off and sleep
        motorFwdLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        sleep(1000);
    }
}