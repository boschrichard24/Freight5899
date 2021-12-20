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

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    //Servo claw;
    //double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        left_Back_Drive = hardwareMap.get(DcMotor.class, "left_Back_Drive");
        right_Back_Drive = hardwareMap.get(DcMotor.class, "right_Back_Drive");
        left_Front_Drive = hardwareMap.get(DcMotor.class, "left_Front_Drive");
        right_Front_Drive = hardwareMap.get(DcMotor.class, "right_Front_Drive");
        left_Arm_Motor = hardwareMap.get(DcMotor.class, "left_Arm_Motor");
        right_Arm_Motor = hardwareMap.get(DcMotor.class, "right_Arm_Motor");

        //claw = hardwareMap.get(Servo.class, "claw");

        //ducky = hardwareMap.get(DcMotor.class, "ducky");

        // Set the direction for each of the motors \\
        left_Back_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Back_Drive.setDirection(DcMotor.Direction.REVERSE);
        left_Front_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Front_Drive.setDirection(DcMotor.Direction.REVERSE);

        //ducky.setDirection(DcMotorSimple.Direction.FORWARD);

        right_Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        left_Arm_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the start button
        telemetry.addData(">", "Ready to run program.");
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
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            leftPower = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;

            //Arm code
            double armDriveLeft = -gamepad2.left_stick_y;
            double armDriveRight = -gamepad2.right_stick_y;

            armLeftpower = Range.clip(armDriveLeft, -0.5, 0.5);
            armRightpower = Range.clip(armDriveRight, -0.5, 0.5);

            // Send calculated power to wheels
            left_Back_Drive.setPower(leftPower);
            right_Back_Drive.setPower(rightPower);
            left_Front_Drive.setPower(leftPower);
            right_Front_Drive.setPower(rightPower);
            left_Arm_Motor.setPower(armLeftpower);
            right_Arm_Motor.setPower(armRightpower);


            // Show the elapsed game time and wheel power
             telemetry.addData("Motors", "left (%.2f), right (%.2f)", armLeftpower, armRightpower);
             // Display the current value
             telemetry.addData(">", "Press Stop to end test." );
             telemetry.update();
        }

        // Signal done;
        //telemetry.addData("Trigger_Value", triggerValue);         Commented out on 11/23/21
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}

