package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ContServoTest", group="Linear Opmode")
public class ContServoTest extends LinearOpMode{
    protected DcMotor left_Back_Drive = null;
    protected DcMotor right_Back_Drive = null;
    protected DcMotor left_Front_Drive = null;
    protected DcMotor right_Front_Drive = null;
    protected DcMotor left_Arm_Motor = null;
    protected DcMotor right_Arm_Motor = null;
    private double spin = 0.0;
    private ElapsedTime runtime = new ElapsedTime();
    protected CRServo spinnerIntake = null;
    private boolean changed1 = false;
    private boolean changed2 = false;


    public void hardwareSetup()
    {
        // Connect Motors to Phone \\
        left_Back_Drive = hardwareMap.get(DcMotor.class, "left_Back_Drive");
        right_Back_Drive = hardwareMap.get(DcMotor.class, "right_Back_Drive");
        left_Front_Drive = hardwareMap.get(DcMotor.class, "left_Front_Drive");
        right_Front_Drive = hardwareMap.get(DcMotor.class, "right_Front_Drive");
        left_Arm_Motor = hardwareMap.get(DcMotor.class, "left_Arm_Motor");
        right_Arm_Motor = hardwareMap.get(DcMotor.class, "right_Arm_Motor");

        spinnerIntake = hardwareMap.get(CRServo.class, "claw");


// Set the direction for each of the motors \\
        left_Back_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Back_Drive.setDirection(DcMotor.Direction.REVERSE);
        left_Front_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Front_Drive.setDirection(DcMotor.Direction.REVERSE);
        right_Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        left_Arm_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        spinnerIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() {

        hardwareSetup();

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad2.a && !changed1){
                spinnerIntake.setPower(5);
                changed1 = true;
            }
            else if(!gamepad2.a){
                changed1 = false;
                spinnerIntake.setPower(0);
            }

            if(gamepad2.b && !changed2){
                spinnerIntake.setPower(-5);
                changed2 = true;
            }
            else if(!gamepad2.b){
                changed2 = false;
                spinnerIntake.setPower(0);
            }
        }
    }


}
