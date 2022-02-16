package org.firstinspires.ftc.teamcode.AutoCode;

//imports

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public abstract class AutoSupplies extends LinearOpMode{

                        //             < < < [ VARIABLES BEGIN HERE ] > > >             \\
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

    protected RevBlinkinLedDriver lights;
    protected CRServo basket = null;  // This is the open and close servo of the claw \\
    protected DcMotor ducky = null;
/*
    protected Rev2mDistanceSensor distanceFwdLeft = null;
    protected Rev2mDistanceSensor distanceFwdRight = null;
    protected Rev2mDistanceSensor distanceBackLeft = null;
    protected Rev2mDistanceSensor distanceBackRight = null;
    protected Rev2mDistanceSensor distanceLeftTop = null;
    protected Rev2mDistanceSensor distanceLeftBottom = null;
*/
    //Encoder Values
    //Neverest 40 motor spec: quadrature encoder, 7 pulses per revolution, count = 7 * 40
    private static final double COUNTS_PER_MOTOR_REV = 420; // Neverest 40 motor encoder - orginal val = 280
    private static final double DRIVE_GEAR_REDUCTION = 1; // This is < 1 if geared up
    private static final double COUNTS_PER_DEGREE1 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 360;
    //---callable methods---\\

    /*public void setup()
    {
        resetArmEncoders();
    }*/

    private static final double A_Left = 1;
    private static final double A_Top = 1; //Fill in with legit values
    private static final double A_Right = 320;
    private static final double A_Bottom = 480;

    private static final double B_Left = 321;
    private static final double B_Top = 1; //Fill in with legit values
    private static final double B_Right = 640;
    private static final double B_Bottom = 480;

    private static final double C_Left = 0;
    private static final double C_Top = 0; //Fill in with legit values
    private static final double C_Right = 0;
    private static final double C_Bottom = 0;


    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Ducky",
            "Marker"
    };


    private static final String VUFORIA_KEY =
            "Ac8qVVb/////AAABmW0ZY5qaKUVegMYq2LOSDO1OzcyAP6IoQTVXJ5E6V+Xier9dD5quzzS0toHeXCyiWZn6Wsw2WdgS9GLwIjNfmuozNwBTuU9DBkABBpyBwAXiiZmzTgLLkNR1dw9+Vwl/S76TuqcaNHTl8vvQOTssFkIvXC0f5acepwlTL8xjEsvb3Y6Fys/mMQprOuhg/9f44K5DsQwutOaTrsVjGyJ1fWyT6cDM+BPqLcBs+/oisbHud/8Q8Iz3I/9+xXJW1ZChn659VoZ0a2Sdoa5FdLl72OpVEzA+d+lYaGcZXmE8NszlxxdOivvNkcFfF45zRyqisSfGowjpyFglNBSWTsNiD1shkpP0uyoeK9lRVxIE4Qug";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

                        // //             < < < [ FUNCTIONS BEGIN HERE ] > > >             \\ \\

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

    public void encoderMove(double degrees, double lp, double rp){
        resetDriveEncoders();
        double counts = degrees * COUNTS_PER_DEGREE1;

        double leftFrontPower = lp;
        double rightFrontPower = rp;
        double leftBackPower = lp;
        double rightBackPower = rp;
        double maxPower;
        double posPower = 0.2;
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
        double averageEnc = (Math.abs(left_Front_Drive.getCurrentPosition())
                + Math.abs(right_Front_Drive.getCurrentPosition())
                + Math.abs(left_Back_Drive.getCurrentPosition())
                + Math.abs(right_Back_Drive.getCurrentPosition()))/4.0;
        while (opModeIsActive() && averageEnc <= counts){
            averageEnc = (Math.abs(left_Front_Drive.getCurrentPosition())
                    + Math.abs(right_Front_Drive.getCurrentPosition())
                    + Math.abs(left_Back_Drive.getCurrentPosition())
                    + Math.abs(right_Back_Drive.getCurrentPosition()))/4.0;
            if(posPower < 1 && averageEnc/counts < .6){
                posPower *= 1.1;
            }
            else if(posPower >= 1 && averageEnc/counts <.6){
                posPower = 1;
            }
            else if(averageEnc/counts >= .6 && posPower >= .25){
                posPower *= .99;
            }
            else{
                posPower = .25;
            }
            left_Front_Drive.setPower(leftFrontPower*posPower);
            left_Back_Drive.setPower(leftBackPower*posPower);
            right_Front_Drive.setPower(rightFrontPower*posPower);
            right_Back_Drive.setPower(rightBackPower*posPower);
        }
        left_Front_Drive.setPower(0);
        left_Back_Drive.setPower(0);
        right_Front_Drive.setPower(0);
        right_Back_Drive.setPower(0);
    }
    
    public void alternateEncoderMove(double degrees, double lp, double rp){
        resetDriveEncoders();
        double counts = degrees * COUNTS_PER_DEGREE1;

        double leftPower = lp;
        double rightPower = rp;
        
        // clamping values between 1 and -1 \\
        if (rightPower > 1.0) { rightPower = 1.0; }
        if (rightPower < -1.0) { rightPower = -1.0; }
        if (leftPower > 1.0) { leftPower = 1.0; }
        if (leftPower < -1.0) { leftPower = -1.0; }
        
        // "If the power is negative and degrees is positive..." \\
        // "...then make the power positive." \\
        if (rightPower < 0 && counts > 0) { rightPower = Math.abs(rightPower); }        // This is really just here for convinience. \\
        if (rightPower > 0 && counts < 0) { rightPower = (Math.abs(rightPower)) * -1; } // You can put a power regardless of the target degree (as in \\
        if (leftPower < 0 && counts > 0) { leftPower = Math.abs(leftPower); }           // you can give a negative degree and positive power) \\
        if (leftPower > 0 && counts < 0) { leftPower = (Math.abs(leftPower)) * -1; }    
                                                                                           
        // this is the average encoder value of all the motors \\
        double averageEnc = (Math.abs(left_Front_Drive.getCurrentPosition())
                + Math.abs(right_Front_Drive.getCurrentPosition())
                + Math.abs(left_Back_Drive.getCurrentPosition())
                + Math.abs(right_Back_Drive.getCurrentPosition()))/4.0;
        
        // "While the robot is running and the average encoder values don't exceed the required encoder target..." \\
        while (opModeIsActive() && averageEnc <= Math.abs(counts)){
            averageEnc = (Math.abs(left_Front_Drive.getCurrentPosition())
                    + Math.abs(right_Front_Drive.getCurrentPosition())
                    + Math.abs(left_Back_Drive.getCurrentPosition())
                    + Math.abs(right_Back_Drive.getCurrentPosition()))/4.0;
            
            left_Front_Drive.setPower(leftPower);
            left_Back_Drive.setPower(leftPower);
            right_Front_Drive.setPower(rightPower);
            right_Back_Drive.setPower(rightPower);
        }
        
        // Don't leave the motors on: \\
        left_Front_Drive.setPower(0);
        left_Back_Drive.setPower(0);
        right_Front_Drive.setPower(0);
        right_Back_Drive.setPower(0);
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

    public void resetDriveEncoders(){
        left_Front_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_Front_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_Back_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_Back_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_Front_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Front_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_Back_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Back_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDrivePower(double lp, double rp)
    {
        double leftFrontPower = lp;
        double rightFrontPower = rp;
        double leftBackPower = lp;
        double rightBackPower = rp;

        left_Front_Drive.setPower(leftFrontPower);
        right_Front_Drive.setPower(rightFrontPower);
        left_Back_Drive.setPower(leftBackPower);
        right_Back_Drive.setPower(rightBackPower);
    }

    public void runArmPower(double power)
    {
        left_Arm_Motor.setPower(power);
        right_Arm_Motor.setPower(power);
    }

    public void runPivotPower(double power)
    {
        pivot_Arm_Motor.setPower(power);
    }

    public void BasketIn(){
        basket.setPower(5);
    }
    public void BasketOut(){
        basket.setPower(-5.0);
    }


    public void resetArmEncoders()
    {
        // Reset all encoders and set their modes \\
        left_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetPivotEncoders()
    {
        pivot_Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot_Arm_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void setArmEncoderMode()
    {
        // Set the mode of the encoders to RUN_TO_POSITION \\
        left_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPivotEncoderMode()
    {
        pivot_Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }




    public void duckyMotorPower(char let)
    {
        double duckyPower = 0.7;

        if (let == 'B') {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE);
            long millis = 1000;
            runtime.reset();
            while(runtime.milliseconds() <= millis){
                ducky.setPower(duckyPower);
            }
            millis = 800;
            runtime.reset();
            while(runtime.milliseconds() <= millis){
                ducky.setPower(duckyPower+1.5);
            }
            ducky.setPower(0);
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

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */


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
//sensors
        /*
        distanceFwdLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceFwdLeft");
        distanceFwdRight = hardwareMap.get(Rev2mDistanceSensor.class, "distanceFwdRight");
        distanceBackLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceBackLeft");
        distanceBackRight = hardwareMap.get(Rev2mDistanceSensor.class, "distanceBackRight");
        distanceLeftTop = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeftTop");
        distanceLeftBottom = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeftBottom");
         */

        basket = hardwareMap.get(CRServo.class, "basket");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

// Set the direction for each of the motors \\
        left_Back_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Back_Drive.setDirection(DcMotor.Direction.REVERSE);
        left_Front_Drive.setDirection(DcMotor.Direction.FORWARD);
        right_Front_Drive.setDirection(DcMotor.Direction.REVERSE);
        right_Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        left_Arm_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot_Arm_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        ducky.setDirection(DcMotorSimple.Direction.FORWARD);

        basket.setDirection(CRServo.Direction.FORWARD);

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

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public void initVision(){
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }
    }

    public Recognition getDuckPosition(){
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    if(recognition.getLabel() == "Ducky") {
                        return recognition;
                    }
                    i++;
                }
                //telemetry.update();
            }
        }
        return null;
    }

    public int getZone(Recognition ducky){
        if(ducky == null){
            return 3;
        }
        double avgPosition = (ducky.getLeft() + ducky.getRight()) / 2;
        if(avgPosition >= A_Left && avgPosition <= A_Right){
            return 1;
        }
        else if(avgPosition >= B_Left && avgPosition <= B_Right){
            return 2;
        }
        else{
            return -1;
        }
    }

                       // //             < < < [ DIST SENSOR CODE BEGIN HERE ] > > >             \\ \\

    /*
    public double getDistanceFwdLeft(){
        return distanceFwdLeft.getDistance(DistanceUnit.MM);
    }
    public double getDistanceFwdRight(){
        return distanceFwdRight.getDistance(DistanceUnit.MM);
    }
    public double getDistanceBackLeft(){
        return distanceBackRight.getDistance(DistanceUnit.MM);
    }
    public double getDistanceBackRight(){
        return distanceBackRight.getDistance(DistanceUnit.MM);
    }

    public double getDistanceLeftTop(){ return distanceLeftTop.getDistance(DistanceUnit.MM); }
    public double getDistanceLeftBottom(){ return distanceLeftBottom.getDistance(DistanceUnit.MM); }

    public void moveUsingLeftDistance(double distance, double power){//- means strafe left and + means strafe right
        while (opModeIsActive()) {
            double dist = distanceLeftTop.getDistance(DistanceUnit.MM);
            double dist2 = distanceLeftBottom.getDistance(DistanceUnit.MM);
            //time out check
            if(dist == 65535 || distanceLeftTop.didTimeoutOccur()) {
                dist = 65535;
            }
            if(dist2 == 65535 || distanceLeftBottom.didTimeoutOccur()){
                dist2 = 65535;
            }
            if(dist >= 8000 || dist2 >= 8000){
                dist = 8190;
                dist2 = 8190;
            }
            //telemetry
            telemetry.addData("distance val 1", dist);
            telemetry.addData("distance val 2", dist2);
            //power set or loop break if distance traveled is met
            if(power <= 0) {
                if (dist != 65535 && dist2 != 65535 && (dist + dist2) / 2 > distance) {
                    setPower(power, 0);
                } else if (dist != 65535 && dist2 == 65535 && dist > distance) {
                    setPower(power, 0);
                } else if (dist == 65535 && dist2 != 65535 && dist2 > distance) {
                    setPower(power, 0);
                } else {
                    break;
                }
            }
            else {
                if (dist != 65535 && dist2 != 65535 && (dist + dist2) / 2 < distance) {
                    setPower(power, 0);
                } else if (dist != 65535 && dist2 == 65535 && dist < distance) {
                    setPower(power, 0);
                } else if (dist == 65535 && dist2 != 65535 && dist2 < distance) {
                    setPower(power, 0);
                } else {
                    break;
                }
            }
            telemetry.update();
        }
    }
    public void moveUsingFwdDistance(double distance, double power){//- means strafe back and + means strafe fwd
        while (opModeIsActive()) {
            double dist = distanceFwdLeft.getDistance(DistanceUnit.MM);
            double dist2 = distanceFwdRight.getDistance(DistanceUnit.MM);
            //time out check
            if(dist == 65535 || distanceFwdLeft.didTimeoutOccur()) {
                dist = 65535;
            }
            if(dist2 == 65535 || distanceFwdRight.didTimeoutOccur()){
                dist2 = 65535;
            }
            if(dist >= 8190 || dist2 >= 8190){
                dist = 8190;
                dist2 = 8190;
            }
            //telemetry
            telemetry.addData("distance val 1", dist);
            telemetry.addData("distance val 2", dist2);
            //power set or loop break if distance traveled is met
            if(power <= 0) {
                if (dist != 65535 && dist2 != 65535 && (dist + dist2) / 2 < distance) {
                    setPower(0, power);
                } else if (dist != 65535 && dist2 == 65535 && dist < distance) {
                    setPower(0, power);
                } else if (dist == 65535 && dist2 != 65535 && dist2 < distance) {
                    setPower(0, power);
                } else {
                    break;
                }
            }
            else {
                if (dist != 65535 && dist2 != 65535 && (dist + dist2) / 2 > distance) {
                    setPower(0, power);
                } else if (dist != 65535 && dist2 == 65535 && dist > distance) {
                    setPower(0, power);
                } else if (dist == 65535 && dist2 != 65535 && dist2 > distance) {
                    setPower(0, power);
                } else {
                    break;
                }
            }
            telemetry.update();
        }
    }
*/
}
