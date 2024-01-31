package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Autonomous2023_24RedLeft", group = "")

public class Autonomous2023_24RedLeft extends LinearOpMode {

    private double yaw;
    private IMU imu;

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    
    private Servo pixelDropper;

    private static final double MOVE_SPEED = 0.5;
    private static final double MAX_LIFT_POSITION = 0;
    private static final double MIN_LIFT_POSITION = -3100.0;
    private static final double MAX_ARM_POSITION = 750.0;
    private static final double MIN_ARM_POSITION = -10000.0;
    //Min and Max of dumpy (servo) are scaled between the right most (max) and left most (min) positions
    private static final double MAX_DUMPY_POSITION = 1.0;
    private static final double MIN_DUMPY_POSITION = 0.6;
    private static final double LEFT_LINE = 150.0;
    private static final double RIGHT_LINE = 300.0;
    private double x;
    private int objectNum = 0;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    

    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/TeamPropRed.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Team Prop",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        motorBackRight = hardwareMap.dcMotor.get("Back_Right");
        
        pixelDropper = hardwareMap.servo.get("pixel_dropper");

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        pixelDropper.setPosition(1.0);

        initTfod();
        waitForStart();

        if (opModeIsActive()) {
            
            yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            
            while (opModeIsActive() && objectNum == 0) {
                telemetryTfod();
            }


            if (x < LEFT_LINE) {
                mecanumMoveBot(-MOVE_SPEED, 0, 0);// movement(F, 0.5, 1 square)
                sleep(500); //Delete
                mecanumMoveBot(0, 0, -MOVE_SPEED);// rotate(??, 90, 0.5)
                sleep(750); 
                mecanumMoveBot(0, 0, 0);
                //Assign variable to this when certain
                pixelDropper.setPosition(0.6);
            }
            else if (x > LEFT_LINE && x < RIGHT_LINE) {
                mecanumMoveBot(-MOVE_SPEED, 0, 0);
                sleep(800);
                mecanumMoveBot(0, 0, 0);
                //Assign variable to this when certain
                pixelDropper.setPosition(0.6);
            }
            else if (x > RIGHT_LINE) {
                mecanumMoveBot(-MOVE_SPEED, 0, 0);
                sleep(500);
                mecanumMoveBot(0, 0, MOVE_SPEED);
                sleep(750);
                mecanumMoveBot(0, 0, 0);
                //Assign variable to this when certain
                pixelDropper.setPosition(0.6);
            }


        }

        // Save CPU resources when camera is no longer needed.
        visionPortal.close();

    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        objectNum = currentRecognitions.size();
        telemetry.addData("# Objects Detected", objectNum);

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.update();
        }

    }

    private void mecanumMoveBot(double FB_translation, double LR_translation, double rotation) {
        double FL_power = FB_translation + LR_translation + rotation;
        double BL_power = FB_translation - LR_translation + rotation;
        double FR_power = FB_translation - LR_translation - rotation;
        double BR_power = FB_translation + LR_translation - rotation;

        motorFrontLeft.setPower(FL_power);
        motorBackLeft.setPower(BL_power);
        motorFrontRight.setPower(FR_power);
        motorBackRight.setPower(BR_power);
    }

    public void movement(char direction, double motorPower, double distance) {
        
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Back_Right");
        
        
        
        if (direction == 'b'){
            distance = -distance;
        }


        double distanceTravelled = 0;
        int numOfTicks = (int)Math.round((distance/(96*Math.PI))*537.7);
        
        
        motorFrontRight.setTargetPosition(numOfTicks);
        motorFrontLeft.setTargetPosition(numOfTicks);
        motorBackRight.setTargetPosition(numOfTicks);
        motorBackLeft.setTargetPosition(numOfTicks);
        
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //distanceTravelled will be calculated with encoders

        /*while (distanceTravelled < distance){
            motorFrontRight.setPower(motorPower);
            motorBackRight.setPower(motorPower);
            motorFrontLeft.setPower(motorPower);
            motorBackLeft.setPower(motorPower);
            distanceTravelled = encoder * 96 * 3.14;
        } */   
        
        motorFrontRight.setPower(motorPower);
        motorBackRight.setPower(motorPower);
        motorFrontLeft.setPower(motorPower);
        motorBackLeft.setPower(motorPower);
        
    }

    public void rotate(char direction, double angle, double motorPower){
        imu.resetYaw();
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Back_Right");
        
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        
        angle = angle-7.5;
        
        if(direction == 'R'){
            while (yaw >= -angle){
                yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                motorFrontLeft.setPower(motorPower);
                motorBackLeft.setPower(motorPower);
                motorFrontRight.setPower(-motorPower);
                motorBackRight.setPower(-motorPower);
                telemetry.addData("yaw", yaw);
                telemetry.update();
            } 
        }
        
        if(direction == 'L'){
            while (yaw <= angle){
                yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                motorFrontLeft.setPower(-motorPower);
                motorBackLeft.setPower(-motorPower);
                motorFrontRight.setPower(motorPower);
                motorBackRight.setPower(motorPower);
                telemetry.addData("yaw", yaw);
                telemetry.update();
            } 
        }
        
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }
}