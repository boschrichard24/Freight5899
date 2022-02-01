package org.firstinspires.ftc.teamcode.AutoCode;

//imports

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class AutoSupplies extends LinearOpMode{
    // Motors, servos, etc.
    protected DcMotor left_Back_Drive = null;
    protected DcMotor right_Back_Drive = null;
    protected DcMotor left_Front_Drive = null;
    protected DcMotor right_Front_Drive = null;
    protected BNO055IMU imu;
    protected DcMotor left_Arm_Motor = null;
    protected DcMotor right_Arm_Motor = null;
    protected DcMotor pivot_Arm_Motor = null;
    protected ElapsedTime runtime = new ElapsedTime();

    //  Protected variables
    protected double globalAngle;
    protected double globalPitch;
    protected Orientation lastAngles = new Orientation();
    protected Orientation lastPitches = new Orientation();


    protected Servo claw = null;  // This is the open and close servo of the claw \\
    protected DcMotor ducky = null;

    // For the claw servo \\
    double lArmMotorEncoderTarget = 0.0; // Angle of arm \\
    double rArmMotorEncoderTarget = 0.0; // Tilt of entire arm


    //---callable methods---\\

    /*public void setup()
    {
        resetArmEncoders();
    }*/

    //move
    public void move(long millis, double lp, double rp) {
        double leftPower = lp;
        double rightPower = rp;
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis) {
            left_Back_Drive.setPower(leftPower);
            left_Front_Drive.setPower(leftPower);
            right_Back_Drive.setPower(rightPower);
            right_Front_Drive.setPower(rightPower);
        }
        left_Back_Drive.setPower(0);
        left_Front_Drive.setPower(0);
        right_Back_Drive.setPower(0);
        right_Front_Drive.setPower(0);
    }


    //Using the gyroscope, when a degree is passed both left and right motors move accordingly in
    //order to turn the robot to the right or left until the bearing is equal to or greater than the
    //specified degree. Power can also be specified.
    //used commonly in pairs(one fast for speed and one slow for accuracy) to improve movement time.
    public void turnToS(int degrees, double power, int loopnum){
        int left  = 1;
        int right = 1;
        double distance = getAngle() - degrees;
        double startAngle = getAngle();
        telemetry.addData("Angle3",getAngle());
        telemetry.update();
        if(getAngle() <= degrees){
            left *= -1;
        }
        else if(getAngle() > degrees){
            right *= -1;
        }

        left_Back_Drive.setPower(power * left);
        left_Front_Drive.setPower(power * left);
        right_Front_Drive.setPower(power * right);
        right_Back_Drive.setPower(power* right);

        if (getAngle() > degrees)
        {
            // On left turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= degrees) {
                //telemetry.addData("Angle4",getAngle());
                //telemetry.update();
                if((startAngle + ((distance/4)*3)) > getAngle()){
                    left *= 1.05;
                    right *= 1.05;
                }
                else{
                    if(left > 1 || left < -1 || right > 1 || right < -1){
                        left*=0.95;
                        right*=0.95;
                    }
                }
            }
        }
        else {    // right turn.
            while (opModeIsActive() && getAngle() <= degrees) {
                //telemetry.addData("Angle4", getAngle());
                //telemetry.update();
                if((startAngle + ((distance/4)*3)) > getAngle()){
                    left *= 1.05;
                    right *= 1.05;
                }
                else{
                    if(left > 1 || left < -1 || right > 1 || right < -1){
                        left*=0.95;
                        right*=0.95;
                    }
                }
            }
        }
        // turn the motors off.
        left_Front_Drive.setPower(0);
        left_Back_Drive.setPower(0);
        right_Back_Drive.setPower(0);
        right_Front_Drive.setPower(0);
        if(--loopnum > 0){
            turnToS(degrees, power/2, loopnum);
        }
    }


    //uses the imu to find the current angle
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }



    public void setArmLevel(int targetLevel)
    {
        int[] encoderTargets = new int[2];

        switch (targetLevel) {
            case 1:
                //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                encoderTargets[0] = -597;
                //sleep(100);
                encoderTargets[1] = -949; // floor level to pick up pieces \\
                break;
            case 2:
                //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                encoderTargets[0] = -432;
                encoderTargets[1] = -842; // level 1 on shipping container \\
                break;
            case 3:
                //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                encoderTargets[0] = -300;
                encoderTargets[1] = -742; // level 2 on shipping container \\
                break;
            case 4:
                //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                encoderTargets[0] = -181;
                encoderTargets[1] = -609; // level 3 on shipping container \\
                break;
            case 5:
                //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
                encoderTargets[0] = -12;
                encoderTargets[1] = -587; // top of shipping container for gamepiece \\
                break;
            case 6:
                //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                encoderTargets[0] = 76;
                encoderTargets[1] = -448; // high as possible (Caed.. we need this?? :\ ) \\
                break;
            case 7:
                //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

                //whatever postiion you want for init
            default:
                //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY);
                encoderTargets[0] = 0;
                encoderTargets[1] = 0; // Default is bottom (level 1) \\
                break;
        }

        runArmPower(.5);
        left_Arm_Motor.setTargetPosition(encoderTargets[0]);
        right_Arm_Motor.setTargetPosition(encoderTargets[1]);
    }



    public void runArmPower(double power)
    {
        left_Arm_Motor.setPower(power);
        right_Arm_Motor.setPower(power);
    }



    public void toggleClaw()
    {
        double clawClosed = 0.363;
        double clawOpen = 0.611;
        double clawPos = 0.0;

        if(clawPos < clawOpen){
            while(clawPos < clawOpen) {
                clawPos += 0.05;
            }
            claw.setPosition(clawPos);

        }
        else if(clawPos > clawClosed){
            while(clawPos > clawClosed) {
                clawPos -= 0.05;
            }
            claw.setPosition(clawPos);
        }
    }



    public void resetArmEncoders()
    {
        // Reset all encoders and set their modes \\
        left_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public void setArmEncoderMode()
    {
        // Set the mode of the encoders to RUN_TO_POSITION \\
        left_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



    public void duckyMotorPower(char let)
    {
        double duckyPower = 0.3;

        if (let == 'B') {
            runtime.reset();
            long millis = 2000;
            runtime.reset();
            while (runtime.milliseconds() <= millis) {
                ducky.setPower(duckyPower);
                duckyPower += 0.001;
            }
            ducky.setPower(0);
            duckyPower = 0.3;
        }

        if (let == 'R') {
            long millis = 2000;
            runtime.reset();
            while (runtime.milliseconds() <= millis) {
                ducky.setPower(-duckyPower);
                duckyPower += 0.001;
            }
            ducky.setPower(0);
            duckyPower = 0.3;
        }
    }



    public void initForAutonomous()
    {
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();

        gyroParameters.mode                = BNO055IMU.SensorMode.IMU;
        gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.loggingEnabled      = false;

// Connect Motors to Phone \\
        left_Back_Drive = hardwareMap.get(DcMotor.class, "left_Back_Drive");
        left_Back_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_Back_Drive = hardwareMap.get(DcMotor.class, "right_Back_Drive");
        right_Back_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_Front_Drive = hardwareMap.get(DcMotor.class, "left_Front_Drive");
        left_Front_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_Front_Drive = hardwareMap.get(DcMotor.class, "right_Front_Drive");
        right_Front_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_Arm_Motor = hardwareMap.get(DcMotor.class, "left_Arm_Motor");
        left_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_Arm_Motor = hardwareMap.get(DcMotor.class, "right_Arm_Motor");
        right_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot_Arm_Motor = hardwareMap.get(DcMotor.class, "pivot_Arm_Motor");
        pivot_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ducky = hardwareMap.get(DcMotor.class, "ducky");
        ducky.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        claw = hardwareMap.get(Servo.class, "claw");
        //lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

// Set the direction for each of the motors \\
        left_Back_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Back_Drive.setDirection(DcMotor.Direction.REVERSE);
        left_Front_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Front_Drive.setDirection(DcMotor.Direction.REVERSE);
        right_Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        left_Arm_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot_Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        ducky.setDirection(DcMotorSimple.Direction.FORWARD);

        left_Back_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_Back_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_Front_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_Front_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot_Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initializes imu and calibrates it. Prepares lift motor to land using the encoder
        // Lights turn green when it is calibrated
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParameters);

        telemetry.clear();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void test()
    {
        //telemetry.update();
    }
}



//  -=={###    OLD AUTOSUPPLIES FOR REFERENCE    ###}==-  \\
/*package org.firstinspires.ftc.teamcode;

//imports




    // This is only for arm tests but maybe not actually i dont know :|
    public void setArmPowers(double mainMotor, double innerAngleMotor, double pivotMotor)
    {
        left_Arm_Motor.setPower(innerAngleMotor);
        right_Arm_Motor.setPower(mainMotor);
        pivot_Arm_Motor.setPower(pivotMotor);
    }


    public void setArmPosition(double power)
    {
        // "While both motors are busy reaching their encoder targets..." \\
        while (right_Arm_Motor.isBusy() || left_Arm_Motor.isBusy()) {
            left_Arm_Motor.setPower(power);
            right_Arm_Motor.setPower(power);
        }
    }

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;


public abstract class AutoSupplies extends LinearOpMode{
    //  Establish hardware
    protected DcMotor  motorFwdLeft   = null;
    protected DcMotor  motorFwdRight  = null;
    protected DcMotor  motorBackLeft  = null;
    protected DcMotor  motorBackRight = null;
    protected BNO055IMU imu;
    protected Rev2mDistanceSensor distanceFwdLeft = null;
    protected Rev2mDistanceSensor distanceFwdRight = null;


    //  Declare OpMode Members
    protected ElapsedTime runtime = new ElapsedTime();
    abstract public void runOpMode() throws InterruptedException;

    //  Protected variables
    protected double globalAngle;
    protected double globalPitch;
    protected Orientation lastAngles = new Orientation();
    protected Orientation lastPitches = new Orientation();

    //---callable methods---

    //move
    public void move(long millis, double x, double y)
    {
        double fwdBackPower = y;
        double strafePower = x;
        double leftFrontPower = fwdBackPower + strafePower;
        double rightFrontPower = fwdBackPower - strafePower;
        double leftBackPower = fwdBackPower - strafePower;
        double rightBackPower = fwdBackPower + strafePower;
        double maxPower;
        double max = 1.0;

        maxPower = Math.abs(leftFrontPower);
        if (Math.abs(rightFrontPower) > maxPower) {
            maxPower = Math.abs(rightFrontPower);
        }
        if (Math.abs(leftBackPower) > maxPower) {
            maxPower = Math.abs(leftBackPower);
        }
        if (Math.abs(rightBackPower) > maxPower) {
            maxPower = Math.abs(rightBackPower);
        }
        if (maxPower > 1) {
            leftFrontPower = leftFrontPower / maxPower;
            rightFrontPower = rightFrontPower / maxPower;
            leftBackPower = leftBackPower / maxPower;
            rightBackPower = rightBackPower / maxPower;

        }
        //sets the power of the motors
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis) {
            motorFwdLeft.setPower(leftFrontPower*max);
            motorFwdRight.setPower(rightFrontPower*max);
            motorBackLeft.setPower(leftBackPower*max);
            motorBackRight.setPower(rightBackPower*max);
        }
        motorFwdLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
    public void setPower(double x, double y)
    {
        double fwdBackPower = y;
        double strafePower = x;
        double leftFrontPower = fwdBackPower + strafePower;
        double rightFrontPower = fwdBackPower - strafePower;
        double leftBackPower = fwdBackPower - strafePower;
        double rightBackPower = fwdBackPower + strafePower;

        motorFwdLeft.setPower(leftFrontPower);
        motorFwdRight.setPower(rightFrontPower);
        motorBackLeft.setPower(leftBackPower);
        motorBackRight.setPower(rightBackPower);
    }
    public void turn(int degrees, double power){
        int left = 1;
        int right = 1;
        resetAngle();
        telemetry.addData("Angle",getAngle());
        telemetry.update();
        if(degrees >= 0){
            right *= -1;
        }
        else if(degrees < 0){
            left *= -1;
        }

        motorBackLeft.setPower(power * left);
        motorFwdLeft.setPower(power * left);
        motorBackRight.setPower(power * right);
        motorFwdRight.setPower(power* right);

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= degrees) {telemetry.addData("Angle2",getAngle());telemetry.update();}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {telemetry.addData("Angle2",getAngle());telemetry.update();}

        // turn the motors off.
        motorBackLeft.setPower(0);
        motorFwdLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFwdRight.setPower(0);
    }
    //Using the gyroscope, when a degree is passed both left and right motors move accordingly in
    //order to turn the robot to the right or left until the bearing is equal to or greater than the
    //specified degree. Power can also be specified.
    //used commonly in pairs(one fast for speed and one slow for accuracy) to improve movement time.
    public void turnToS(int degrees, double power, int loopnum){
        int left = 1;
        int right = 1;
        double distance = getAngle() - degrees;
        double startAngle = getAngle();
        telemetry.addData("Angle3",getAngle());
        telemetry.update();
        if(getAngle() <= degrees){
            left *= -1;
        }
        else if(getAngle() > degrees){
            right *= -1;
        }

        motorBackLeft.setPower(power * left);
        motorFwdLeft.setPower(power * left);
        motorBackRight.setPower(power * right);
        motorFwdRight.setPower(power* right);

        if (getAngle() > degrees)
        {
            // On left turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= degrees) {
                //telemetry.addData("Angle4",getAngle());
                //telemetry.update();
                if((startAngle + ((distance/4)*3)) > getAngle()){
                    left *= 1.05;
                    right *= 1.05;
                }
                else{
                    if(left > 1 || left < -1 || right > 1 || right < -1){
                        left*=0.95;
                        right*=0.95;
                    }
                }
            }
        }
        else {    // right turn.
            while (opModeIsActive() && getAngle() <= degrees) {
                //telemetry.addData("Angle4", getAngle());
                //telemetry.update();
                if((startAngle + ((distance/4)*3)) > getAngle()){
                    left *= 1.05;
                    right *= 1.05;
                }
                else{
                    if(left > 1 || left < -1 || right > 1 || right < -1){
                        left*=0.95;
                        right*=0.95;
                    }
                }
            }
        }
        // turn the motors off.
        motorBackLeft.setPower(0);
        motorFwdLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFwdRight.setPower(0);
        if(--loopnum > 0){
            turnToS(degrees, power/2, loopnum);
        }
    }
    //Using the gyroscope, when a degree is passed both left and right motors move accordingly in
    //order to turn the robot to the right or left until the bearing is equal to or greater than the
    //specified degree. Power can also be specified.
    //used commonly in pairs(one fast for speed and one slow for accuracy) to improve movement time.
    public void turnTo(int degrees, double power){
        int left = 1;
        int right = 1;
        telemetry.addData("Angle3",getAngle());
        telemetry.update();
        if(getAngle() >= degrees){
            left *= -1;
        }
        else if(getAngle() < degrees){
            right *= -1;
        }

        motorBackLeft.setPower(power * left);
        motorFwdLeft.setPower(power * left);
        motorBackRight.setPower(power * right);
        motorFwdRight.setPower(power* right);

        if (getAngle() > degrees)
        {
            // On left turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= degrees) {
                //telemetry.addData("Angle4",getAngle());
                //telemetry.update();
            }
        }
        else {    // right turn.
            while (opModeIsActive() && getAngle() <= degrees) {
                //telemetry.addData("Angle4", getAngle());
                //telemetry.update();
            }
        }
        // turn the motors off.
        motorBackLeft.setPower(0);
        motorFwdLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFwdRight.setPower(0);
    }
    //  Pause for the specified amount of time (time: mili secs)
    public void pause(long millis){
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis){

        }
    }
    //Resets gyro sensor bearing value to 0
    //commonly used to calibrate before a match as well
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    //Resets gyro sensor pitch value to 0
    //commonly used to calibrate before a match as well
    public void resetPitch()
    {
        lastPitches = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalPitch = 0;
    }

    //
    // * Get current cumulative angle rotation from last reset.
    // * @return Angle in degrees. + = left, - = right.
    // *


    //uses the imu to find the current angle
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    //uses the imu to get the current pitch of the robot
    public double getPitch() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.secondAngle - lastPitches.secondAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalPitch += deltaAngle;

        lastPitches = angles;

        return globalPitch;
    }
    public double getDistanceLeft(){
        return distanceFwdLeft.getDistance(DistanceUnit.MM);
    }
    public double getDistanceRight(){
        return distanceFwdRight.getDistance(DistanceUnit.MM);
    }

    public void initForAutonomous()
    {
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();

        gyroParameters.mode                = BNO055IMU.SensorMode.IMU;
        gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.loggingEnabled      = false;

        //initialize hardware
        //main motors
        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
        motorFwdRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
        motorFwdLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFwdLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFwdRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFwdLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //sensors
        distanceFwdLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeft");
        distanceFwdRight = hardwareMap.get(Rev2mDistanceSensor.class, "distanceRight");

        //initializes imu and calibrates it. Prepares lift motor to land using the encoder
        // Lights turn green when it is calibrated
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParameters);

        telemetry.clear();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
*/