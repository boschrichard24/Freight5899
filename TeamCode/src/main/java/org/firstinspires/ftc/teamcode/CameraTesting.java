package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


@TeleOp(name = "Concept: TensorFlow Object Detection", group = "Concept")
@Disabled
public class CameraTesting extends LinearOpMode {

    private static final double A_Left = 0;
    private static final double A_Top = 0; //Fill in with legit values
    private static final double A_Right = 0;
    private static final double A_Bottom = 0;

    private static final double B_Left = 0;
    private static final double B_Top = 0; //Fill in with legit values
    private static final double B_Right = 0;
    private static final double B_Bottom = 0;

    private static final double C_Left = 0;
    private static final double C_Top = 0; //Fill in with legit values
    private static final double C_Right = 0;
    private static final double C_Bottom = 0;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "Ac8qVVb/////AAABmW0ZY5qaKUVegMYq2LOSDO1OzcyAP6IoQTVXJ5E6V+Xier9dD5quzzS0toHeXCyiWZn6Wsw2WdgS9GLwIjNfmuozNwBTuU9DBkABBpyBwAXiiZmzTgLLkNR1dw9+Vwl/S76TuqcaNHTl8vvQOTssFkIvXC0f5acepwlTL8xjEsvb3Y6Fys/mMQprOuhg/9f44K5DsQwutOaTrsVjGyJ1fWyT6cDM+BPqLcBs+/oisbHud/8Q8Iz3I/9+xXJW1ZChn659VoZ0a2Sdoa5FdLl72OpVEzA+d+lYaGcZXmE8NszlxxdOivvNkcFfF45zRyqisSfGowjpyFglNBSWTsNiD1shkpP0uyoeK9lRVxIE4Qug";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVision();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Recognition duck = null;
                while(true){
                    duck = getDuckPosition();
                    if(duck != null){
                        break;
                    }
                }
                telemetry.addData(String.format("  left,top (%d)", 1), "%.03f , %.03f",
                        duck.getLeft(), duck.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", 1), "%.03f , %.03f",
                        duck.getRight(), duck.getBottom());
                telemetry.update();
                sleep(10000);
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
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

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
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
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    /*telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom()); */
                    if(recognition.getLabel() == "Ducky") {
                        return recognition;
                    }
                    i++;
                }
                telemetry.update();
            }
        }
        return null;
    }
}
