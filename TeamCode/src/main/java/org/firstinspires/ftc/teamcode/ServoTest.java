package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ServoTest", group = "TeleOp")

public class ServoTest extends LinearOpMode {
    //All hardware
    protected DcMotor left_Back_Drive   = null;
    protected DcMotor right_Back_Drive  = null;
    protected DcMotor left_Front_Drive  = null;
    protected DcMotor right_Front_Drive = null;
    protected DcMotor left_Arm_Motor    = null;
    protected DcMotor right_Arm_Motor   = null;
    protected DcMotor pivot_Arm_Motor   = null;
    protected DcMotor ducky             = null;
    protected Servo claw        = null;  // This is the open and close servo of the claw \\

    double servo = 0.5;

    @Override
    public void runOpMode() {
        //Prepares all the hardware
        left_Back_Drive = hardwareMap.get(DcMotor.class, "left_Back_Drive");
        right_Back_Drive = hardwareMap.get(DcMotor.class, "right_Back_Drive");
        left_Front_Drive = hardwareMap.get(DcMotor.class, "left_Front_Drive");
        right_Front_Drive = hardwareMap.get(DcMotor.class, "right_Front_Drive");
        left_Arm_Motor = hardwareMap.get(DcMotor.class, "left_Arm_Motor");
        right_Arm_Motor = hardwareMap.get(DcMotor.class, "right_Arm_Motor");
        pivot_Arm_Motor = hardwareMap.get(DcMotor.class, "pivot_Arm_Motor");
        ducky = hardwareMap.get(DcMotor.class, "pivot_Arm_Motor");

        left_Back_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Back_Drive.setDirection(DcMotor.Direction.REVERSE);
        left_Front_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Front_Drive.setDirection(DcMotor.Direction.REVERSE);
        right_Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        left_Arm_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot_Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        ducky.setDirection(DcMotorSimple.Direction.FORWARD);


        claw = hardwareMap.get(Servo.class, "claw_Servo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.dpad_up){
                servo += 0.001;
            }
            else if(gamepad1.dpad_down){
                servo -= 0.001;
            }
            /*if(gamepad1.dpad_left){
                servo2 += 0.001;
            }
            else if(gamepad1.dpad_right){
                servo2 -= 0.001;
            }
             */
            claw.setPosition(servo);
            //ringPullPivotServo.setPosition(servo2);

            //telemetry.addData("unloadServo",wobbleArmServo.getPosition());
            telemetry.addData("servo", servo);
            telemetry.addData("claw", claw);
            /*
            telemetry.addData("Distance Left Top", distanceLeftTop.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance Left Bottom", distanceLeftBottom.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance Forward Right", distanceFwdRight.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance Forward Left", distanceFwdLeft.getDistance(DistanceUnit.MM));
*/
            telemetry.update();
        }
    }
}
