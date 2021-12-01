// Aidan was here
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="SimpleDrive", group="Linear Opmode")
//@Disabled

public class SimpleDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_Back_Drive = null;
    private DcMotor right_Back_Drive = null;
    private DcMotor left_Front_Drive = null;
    private DcMotor right_Front_Drive = null;
    private DcMotor left_Arm_Motor = null;
    private DcMotor right_Arm_Motor = null;
    //private DcMotor ducky = null;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    Servo claw;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        left_Back_Drive  = hardwareMap.get(DcMotor.class, "left_Back_Drive");
        right_Back_Drive = hardwareMap.get(DcMotor.class, "right_Back_Drive");
        left_Front_Drive  = hardwareMap.get(DcMotor.class, "left_Front_Drive");
        right_Front_Drive = hardwareMap.get(DcMotor.class, "right_Front_Drive");
        left_Arm_Motor = hardwareMap.get(DcMotor.class, "left_Arm_Motor");
        right_Arm_Motor = hardwareMap.get(DcMotor.class, "right_Arm_Motor");
        claw = hardwareMap.get(Servo.class, "claw");

       // ducky = hardwareMap.get(DcMotor.class, "ducky");

        // Set the direction for each of the motors \\
        left_Back_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Back_Drive.setDirection(DcMotor.Direction.REVERSE);
        left_Front_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Front_Drive.setDirection(DcMotor.Direction.REVERSE);
        //ducky.setDirection(DcMotorSimple.Direction.FORWARD);
        right_Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        left_Arm_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // THESE ARE FOR TELEMETRY TESTING \\
        /* I used these to test different types of input for the controller.
        * The X, B, Y, and A buttons are used to test stuff like a button toggle,
        * a button hold, a counter, and an option sifter (press a button to sift
        * through a list of options). It displays info on the phone, but it's commented out.
        */

        /*int timesPressed = 0;
        double timeHeld = 0;
        boolean toggle = false;
        int optionSiftIndex = 0;            Commented out on 11/23/21
        String option = "";
        boolean yHasBeenPressed = false;
        boolean aHasBeenPressed = false;

        float triggerValue = 0.0f;*/

        // Wait for the start button
        telemetry.addData(">", "Ready to run program." );
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double leftPower;
        double rightPower;
        double armLeftpower;
        double armRightpower;
        double servoPower;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            //triggerValue = gamepad1.right_trigger;            Commented out on 11/23/21

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            //double drive = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;

            //leftPower = Range.clip(drive + turn, -2.0, 2.0);
            //rightPower = Range.clip(drive - turn, -2.0, 2.0);

            //Servo Move Func \\
            //ducky is input and ducky_run is the toggle \\
            /*if (gamepad1.right_bumper) {
                ducky.setPower(2.5);
            }
            else{
                ducky.setPower(0);
            }
            servo = hardwareMap.get(Servo.class, "left_hand");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();

*/
        // Scan servo till stop pressed.
            boolean rampUp = gamepad2.a;
        //while(opModeIsActive()){

            // slew the servo, according to the rampUp (direction) variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;

                }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;

                }
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            claw.setPosition(position);
            sleep(CYCLE_MS);
            idle();
        //}

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.

            leftPower  = -gamepad1.left_stick_y ;
            rightPower = -gamepad1.right_stick_y ;
            double armDriveLeft = -gamepad2.left_stick_y;
            double armDriveRight = -gamepad2.right_stick_y;
            armLeftpower = Range.clip(armDriveLeft, -0.5, 0.5);
            armRightpower = Range.clip(armDriveRight, -0.5, 0.5);
            // --={####     THIS CODE IS FOR TELEMETRY TESTS    ####}=-- \\
            /*
            if (gamepad1.y) {
                if (yHasBeenPressed = false) timesPressed ++;
                yHasBeenPressed = true;
            }
            else {
                yHasBeenPressed = false;
            }

            if (gamepad1.b) {
                timeHeld ++;
            }
            else {
                timeHeld = 0;
            }

            if (gamepad1.a) {
                toggle = !toggle;
            }

            if (gamepad1.x) {
                optionSiftIndex ++;
                if (optionSiftIndex > 3) optionSiftIndex = 1;
                //if (optionSiftIndex == 1) option = "Option 1";
                //if (optionSiftIndex == 2) option = "Option 2";
                //if (optionSiftIndex == 3) option = "Option 3";
                switch (optionSiftIndex) {
                    case 1:
                        option = "Option 1";
                        break;
                    case 2:
                        option = "Option 2";
                        break;
                    case 3:
                        option = "Option 3";
                        break;
                    default:
                        option = "Somethings wrong :(";
                        break;
                }
            }

            telemetry.addData("y-button (times pressed) || ", timesPressed);
            telemetry.addData("b-button (increase on hold) || ", timeHeld);
            telemetry.addData("a-button (toggle switch) || ", toggle);
            telemetry.addData("x-button (option sifter) || ", option);
            telemetry.update();


             */

            // Send calculated power to wheels
            left_Back_Drive.setPower(leftPower);
            right_Back_Drive.setPower(rightPower);
            left_Front_Drive.setPower(leftPower);
            right_Front_Drive.setPower(rightPower);
            left_Arm_Motor.setPower(armLeftpower);
            right_Arm_Motor.setPower(armRightpower);


            // Show the elapsed game time and wheel power.
            /* telemetry.addData("Status", "Run Time: " + runtime.toString());
             * telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
             * Display the current value
             * telemetry.addData(">", "Press Stop to end test." );
             * telemetry.update();
             */
        }

        // Signal done;
        //telemetry.addData("Trigger_Value", triggerValue);         Commented out on 11/23/21
        //telemetry.addData(">", "Done");
       // telemetry.update();
    }
