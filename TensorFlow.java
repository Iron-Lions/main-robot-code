package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "TensorFlow", group = "")

public class TensorFlow extends LinearOpMode {

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    private DcMotor lift;
    private DcMotor arm;
    private DcMotor intake;
    private Servo dumpy_4;

    private static final double MOVE_SPEED = 0.5;
    private static final double MAX_LIFT_POSITION = 0;
    private static final double MIN_LIFT_POSITION = -3100.0;
    private static final double MAX_ARM_POSITION = 750.0;
    private static final double MIN_ARM_POSITION = -10000.0;
    //Min and Max of dumpy (servo) are scaled between the right most (max) and left most (min) positions
    private static final double MAX_DUMPY_POSITION = 1.0;
    private static final double MIN_DUMPY_POSITION = 0.5;
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
        intake = hardwareMap.dcMotor.get("intake");
        lift = hardwareMap.dcMotor.get("lift");
        arm = hardwareMap.dcMotor.get("arm");
        dumpy_4 = hardwareMap.servo.get("dumpy_4");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        initTfod();
        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive() && objectNum == 0) {
                telemetryTfod();
            }


            if (x < LEFT_LINE) {
                mecanumMoveBot(MOVE_SPEED, 0, 0);
                sleep(1200); // Tuning
                mecanumMoveBot(0, 0, MOVE_SPEED);
                sleep(940); // Tuning
                mecanumMoveBot(0, 0, 0);
                // Drop Pixel

            }
            if (x > LEFT_LINE && x < RIGHT_LINE) {
                mecanumMoveBot(MOVE_SPEED, 0, 0);
                sleep(1200); // Tuning
                mecanumMoveBot(0, 0, 0);
                //Drop pixel
            }
            if (x > RIGHT_LINE) {
                mecanumMoveBot(MOVE_SPEED, 0, 0);
                sleep(1200); // Tuning
                mecanumMoveBot(0, 0, -MOVE_SPEED);
                sleep(940); // Tuning
                mecanumMoveBot(0, 0, 0);
                // Drop pixel
            }

        }
        visionPortal.close();

    }

    private void initTfod() {

        tfod = new TfodProcessor.Builder()

                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.enableLiveView(true);
        builder.addProcessor(tfod);

        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.5f);
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

    private void armMovement(double power, int targetPosition) {
        if (targetPosition >= MAX_ARM_POSITION) {
            arm.setTargetPosition((int)MAX_ARM_POSITION);
            arm.setPower(power);
        }
        if (targetPosition <= MIN_ARM_POSITION) {
            arm.setTargetPosition((int) MIN_ARM_POSITION);
            arm.setPower(power);
        }
        else {
            arm.setTargetPosition(targetPosition);
            arm.setPower(power);
        }
    }

    private void liftMovement(double power, int targetPosition) {
        if (targetPosition >= MAX_LIFT_POSITION) {
            lift.setTargetPosition((int) MAX_LIFT_POSITION);
            lift.setPower(power);
        }
        if (targetPosition <= MIN_LIFT_POSITION) {
            lift.setTargetPosition((int) MIN_LIFT_POSITION);
            lift.setPower(power);
        }
        else {
            lift.setTargetPosition(targetPosition);
            lift.setPower(power);
        }
    }

    private void intakeMovement(double power, double setTime) { //setTime is in ms
        intake.setPower(power);
        sleep((long) setTime);
        intake.setPower(0);
    }

    private void dumpy_4Movement(double targetPosition) {
        if (targetPosition >= MAX_DUMPY_POSITION) {
            dumpy_4.setPosition(MAX_DUMPY_POSITION);
        }
        else {
            dumpy_4.setPosition(Math.max(targetPosition, MIN_DUMPY_POSITION));
        }
    }

}
