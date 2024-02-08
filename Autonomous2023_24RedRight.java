package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
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

@Autonomous(name = "Autonomous2023_24RedRight", group = "")

public class Autonomous2023_24RedRight extends LinearOpMode {

    private double yaw;
    private IMU imu;

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    private Servo pixelDropper;
    private Servo backdrop;

    private static final double MOVE_SPEED = 0.25;
    private static final double MAX_LIFT_POSITION = 0;
    private static final double MIN_LIFT_POSITION = -3100.0;
    private static final double MAX_ARM_POSITION = 750.0;
    private static final double MIN_ARM_POSITION = -10000.0;
    //Min and Max of dumpy (servo) are scaled between the right most (max) and left most (min) positions
    private static final double MAX_PIXELDROPPER_POSITION = 0.85;
    private static final double MIN_PIXELDROPPER_POSITION = 0.65;
    private static final double INIT_BACKDROP_POSITION = 1;
    private static final double READY_BACKDROP_POSITION = 0.6;
    private static final double FINISH_BACKDROP_POSITION = 0;
    private static final double MIDDLE_LINE = 250.0;
    private double x;
    private int objectNum = 0;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera


    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/TeamPropRed.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Red Team Prop",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    private boolean isLeft = false;
    private boolean isRight = false;
    private boolean isMiddle = false;

    private ElapsedTime runTime = new ElapsedTime();

    @Override
    public void runOpMode() {

        motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        motorBackRight = hardwareMap.dcMotor.get("Back_Right");

        pixelDropper = hardwareMap.servo.get("pixel_dropper");
        backdrop = hardwareMap.servo.get("backdrop");
        //pixelDropper.setPosition(MIN_DUMPY_POSITION);
        imu = hardwareMap.get(IMU.class, "imu");


        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        imu.resetYaw();

        initTfod();
        waitForStart();

        if (opModeIsActive()) {

            pixelDropper.setPosition(MIN_PIXELDROPPER_POSITION);
            backdrop.setPosition(FINISH_BACKDROP_POSITION);
            yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if (opModeIsActive() && objectNum == 0) { // Check Middle
                runTime.reset();
                while (opModeIsActive() && runTime.milliseconds() <= 3000) {
                    telemetryTfod();
                }

                if (objectNum == 1 && x > MIDDLE_LINE) { // Middle Movement
                    mecanumMoveBotEncoders(MOVE_SPEED, 0, 0, .75);
                    //*drop pixel*
                    pixelDropper.setPosition(MIN_PIXELDROPPER_POSITION);
                    mecanumMoveBotEncoders(-MOVE_SPEED, 0, 0, .25);
                    Rotate('R', 90, 0.5);
                    mecanumMoveBotEncoders(MOVE_SPEED, 0, 0, .75);
                    //*flip forward backdrop servo*
                    backdrop.setPosition(READY_BACKDROP_POSITION);
                    //*Align with center april tag*
                    mecanumMoveBotEncoders(MOVE_SPEED, 0, 0, .3);
                    mecanumMoveBotEncoders(-MOVE_SPEED, 0, 0, .1);
                    //*flip down backdrop servo*
                    backdrop.setPosition(FINISH_BACKDROP_POSITION);
                    Rotate('R', 90, 0.5);
                    mecanumMoveBotEncoders(MOVE_SPEED, 0, 0, .8);
                    //**End of program**

                }
                if (objectNum == 1 && x < MIDDLE_LINE) { // Left Movement
                    mecanumMoveBotEncoders(MOVE_SPEED, 0, 0,.65);
                    Rotate('L', 90, MOVE_SPEED);
                    mecanumMoveBotEncoders(MOVE_SPEED, 0, 0, .1);
                    //*drop pixel*
                    pixelDropper.setPosition(MAX_PIXELDROPPER_POSITION);
                    mecanumMoveBotEncoders(-MOVE_SPEED, 0, 0,.1);
                    Rotate('R', 180, MOVE_SPEED);
                    mecanumMoveBotEncoders(MOVE_SPEED, 0, 0, .65);
                    //*flip forward backdrop servo*
                    backdrop.setPosition(READY_BACKDROP_POSITION);
                    //Align with left april tag
                    mecanumMoveBotEncoders(.1, 0, 0,.3);
                    mecanumMoveBotEncoders(-MOVE_SPEED, 0, 0, .1);
                    //*flip down backdrop servo*
                    backdrop.setPosition(FINISH_BACKDROP_POSITION);
                    Rotate('R', 90, MOVE_SPEED);
                    mecanumMoveBotEncoders(MOVE_SPEED, 0, 0, .3);
                    //**End of program*

                }
                if (objectNum == 0) { // Right
                    mecanumMoveBotEncoders(MOVE_SPEED, 0, 0, .25);
                    Rotate('R', 90, 0.5);
                    mecanumMoveBotEncoders(MOVE_SPEED, 0, 0, .25);
                    Rotate('L', 90, 0.5);
                    mecanumMoveBotEncoders(MOVE_SPEED, 0, 0,.75);
                    mecanumMoveBotEncoders(-MOVE_SPEED, 0, 0,.25);
                    //*drop pixel*
                    pixelDropper.setPosition(MAX_PIXELDROPPER_POSITION);
                    mecanumMoveBotEncoders(-MOVE_SPEED, 0, 0, .1);
                    Rotate('R', 90, 0.5);
                    //*flip forward backdrop servo*
                    backdrop.setPosition(READY_BACKDROP_POSITION);
                    //*Align with right april tag*
                    mecanumMoveBotEncoders(MOVE_SPEED, 0, 0,.3);
                    mecanumMoveBotEncoders(-MOVE_SPEED, 0, 0, .1);
                    //*flip down backdrop servo*
                    backdrop.setPosition(FINISH_BACKDROP_POSITION);
                    Rotate('R', 90, 0.5);
                    mecanumMoveBotEncoders(MOVE_SPEED, 0, 0, .8);
                    //**End of program**

                }
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
        tfod.setMinResultConfidence(0.8f);

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

    //Distance is in meters
    private void mecanumMoveBotEncoders(double FB_translation, double LR_translation, double rotation, double distance) {
        // Calculate individual motor powers
        FB_translation = -FB_translation;
        double FL_power = FB_translation + LR_translation + rotation;
        double BL_power = FB_translation - LR_translation + rotation;
        double FR_power = FB_translation - LR_translation - rotation;
        double BR_power = FB_translation + LR_translation - rotation;

        int FL_direction = 0;
        int BL_direction = 0;
        int FR_direction = 0;
        int BR_direction = 0;

        distance = distance * 1000; //meters to mm

        distance = distance * 0.97035; // Tune ratio

        double numberOfTicks = (distance/(96*Math.PI))*537.7;


        if (FL_power > 0) {
            FL_direction = 1;
        }
        else if (FL_power < 0) {
            FL_direction = -1;
        }
        if (BL_power > 0) {
            BL_direction = 1;
        }
        else if (BL_power < 0) {
            BL_direction = -1;
        }
        if (FR_power > 0) {
            FR_direction = 1;
        }
        else if (FR_power < 0) {
            FR_direction = -1;
        }
        if (BR_power > 0) {
            BR_direction = 1;
        }
        else if (BR_power < 0) {
            BR_direction = -1;
        }

        // Set target encoder positions
        int BLTargetPosition = motorBackLeft.getCurrentPosition() + (int) (numberOfTicks * (BL_direction));
        int FLTargetPosition = motorFrontLeft.getCurrentPosition() + (int) (numberOfTicks * (FL_direction));
        int FRTargetPosition = motorFrontRight.getCurrentPosition() + (int) (numberOfTicks * (FR_direction));
        int BRTargetPosition = motorBackRight.getCurrentPosition() + (int) (numberOfTicks * (BR_direction));

        // Set target positions for encoders
        motorFrontLeft.setTargetPosition(FLTargetPosition);
        motorBackLeft.setTargetPosition(BLTargetPosition);
        motorFrontRight.setTargetPosition(FRTargetPosition);
        motorBackRight.setTargetPosition(BRTargetPosition);

        // Set motor power and run to position
        motorFrontLeft.setPower(FL_power);
        motorBackLeft.setPower(BL_power);
        motorFrontRight.setPower(FR_power);
        motorBackRight.setPower(BR_power);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait until all motors reach their target position
        while (opModeIsActive() && motorFrontLeft.isBusy() && motorBackRight.isBusy() && motorBackLeft.isBusy() && motorFrontRight.isBusy()) {
            telemetry.addData("FrontRight Position", motorFrontRight.getCurrentPosition());
            telemetry.addData("BackLeft Position", motorBackLeft.getCurrentPosition());
            telemetry.addData("FrontLeft Position", motorFrontLeft.getCurrentPosition());
            telemetry.addData("BackRight Position", motorBackRight.getCurrentPosition());
            telemetry.addData("FrontRight Target", motorFrontRight.getTargetPosition());
            telemetry.addData("BackLeft Target", motorBackLeft.getTargetPosition());
            telemetry.addData("FrontLeft Target", motorFrontLeft.getTargetPosition());
            telemetry.addData("BackRight Target", motorBackRight.getTargetPosition());
            telemetry.update();
        }

        // Stop motors
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors back to using encoders
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
    }

    public void Rotate(char direction, double angle, double motorPower){
        imu.resetYaw();
        yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Back_Right");

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        angle = (angle - 7.5) * 1; // Tuning 1 if needed

        if (direction == 'R') {
            while (yaw >= -angle) {
                yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                motorFrontLeft.setPower(-motorPower);
                motorBackLeft.setPower(-motorPower);
                motorFrontRight.setPower(motorPower);
                motorBackRight.setPower(motorPower);
                telemetry.addData("yaw", yaw);
                telemetry.update();
            }
        }

        if (direction == 'L') {
            while (yaw <= angle) {
                yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                motorFrontLeft.setPower(motorPower);
                motorBackLeft.setPower(motorPower);
                motorFrontRight.setPower(-motorPower);
                motorBackRight.setPower(-motorPower);
                telemetry.addData("yaw", yaw);
                telemetry.update();
            }
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        telemetry.addData("Encoder Value", motorFrontRight.getCurrentPosition());
        telemetry.addData("Target Value", motorFrontRight.getTargetPosition());
        telemetry.update();
        sleep(500);
    }
}
