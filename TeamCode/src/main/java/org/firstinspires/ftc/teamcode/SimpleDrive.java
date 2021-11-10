package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;


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

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_Back_Drive = null;
    private DcMotor right_Back_Drive = null;
    private DcMotor left_Front_Drive = null;
    private DcMotor right_Front_Drive = null;
    // amount to slew servo each CYCLE_MS cycle
    // Define class members
    CRServo   ducky;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo   servo;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;

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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        left_Back_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Back_Drive.setDirection(DcMotor.Direction.REVERSE); //hherhigrgeg
        left_Front_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Front_Drive.setDirection(DcMotor.Direction.REVERSE);

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        ducky = hardwareMap.get(CRServo.class, "ducky");

        // THESE ARE FOR TELEMETRY TESTING \\
        int timesPressed = 0;
        double timeHeld = 0;
        boolean toggle = false;
        int optionSiftIndex = 0;
        String option = "";
        boolean yHasBeenPressed = false;
        boolean aHasBeenPressed = false;

        // Wait for the start button
        telemetry.addData(">", "Ready to run program." );
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Connect to servo (Assume PushBot Left Hand)
            // Change the text in quotes to match any servo name on your robot.
            servo = hardwareMap.get(Servo.class, "hand");

            // Wait for the start button
            telemetry.addData(">", "Press Start to scan Servo." );
            telemetry.update();
            waitForStart();


            // Scan servo till stop pressed.
            while(opModeIsActive()){

                // slew the servo, according to the rampUp (direction) variable.
                if(gamepad1.x) {
                    if (rampUp) {
                        // Keep stepping up until we hit the max value.
                        position += INCREMENT;
                        if (position >= MAX_POS) {
                            position = MAX_POS;
                            rampUp = !rampUp;   // Switch ramp direction
                        }
                    } else {
                        // Keep stepping down until we hit the min value.
                        position -= INCREMENT;
                        if (position <= MIN_POS) {
                            position = MIN_POS;
                            rampUp = !rampUp;  // Switch ramp direction
                        }
                    }
                }

                // Display the current value
                telemetry.addData("Servo Position", "%5.2f", position);
                telemetry.addData(">", "Press Stop to end test." );
                telemetry.update();

                // Set the servo to the new position and pause;
                servo.setPosition(position);
                sleep(CYCLE_MS);
                idle();
            }

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double servoPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            // double drive = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;

            //leftPower    = Range.clip(drive + turn, -2.0, 2.0) ;
            //rightPower   = Range.clip(drive - turn, -2.0, 2.0) ;

            //Servo Move Func
            //ducky is input and ducky_run is the toggle

            if (gamepad1.right_bumper) {
                ducky.setDirection(DcMotorSimple.Direction.FORWARD);
                //ducky.setPower(2.5);
            }
            else{
                ducky.setDirection(DcMotorSimple.Direction.FORWARD);
                //ducky.setPower(0);
            }

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            leftPower  = -gamepad1.left_stick_y ;
            rightPower = -gamepad1.right_stick_y ;

            // --={####     THIS CODE IS FOR TELEMETRY TESTS    ####}=-- \\

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

            // --={####     UNCOMMENT THIS CODE!!!   ####}=-- \\
            // uncomment lines 97 and 101 too \\

            // Send calculated power to wheels
            //left_Back_Drive.setPower(leftPower);
            //right_Back_Drive.setPower(rightPower);
            //left_Front_Drive.setPower(leftPower);
            //right_Front_Drive.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            /* telemetry.addData("Status", "Run Time: " + runtime.toString());
             * telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
             * // Display the current value
             * telemetry.addData("Servo Position", "%5.2f", position);
             * telemetry.addData(">", "Press Stop to end test." );
             * telemetry.update();
             */
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
