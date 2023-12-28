package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Autonomous2023_24", group = "")
public class Autonomous2023_24 extends LinearOpMode {

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
    private static final double MAX_DUMPY_POSITION = 1.0;
    private static final double MIN_DUMPY_POSITION = 0.5;
    private static final double LEFT_LINE = 150.0;
    private static final double RIGHT_LINE = 300.0;
    private double x;
    private int objectNum = 0;
    private static final boolean USE_WEBCAM = true;
    private WebcamName webcam1;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/TeamPropRed.tflite";
    private static final String[] LABELS = {
            "Team Prop",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private static final double DESIRED_DISTANCE = 12.0;
    private static final double APRIL_TAG_SPEED_GAIN = 0.02;
    private static final double APRIL_TAG_STRAFE_GAIN = 0.015;
    private static final double APRIL_TAG_TURN_GAIN = 0.01;
    private static final double APRIL_TAG_MAX_AUTO_SPEED = 0.5;
    private static final double APRIL_TAG_MAX_AUTO_STRAFE = 0.5;
    private static final double APRIL_TAG_MAX_AUTO_TURN = 0.3;
    private static int desiredTagId = -1;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private int detectedTagId;
    private boolean targetFound = false;
    private double drive = 0;
    private double strafe = 0;
    private double turn = 0;
    double rangeError = -1;
    private double headingError = -1;
    private double yawError = -1;

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
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        initTfod();
        initAprilTag();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive() && objectNum == 0) {
                telemetryTfod();
            }

            if (x < LEFT_LINE) {
                desiredTagId = 4;
                mecanumMoveBot(0, MOVE_SPEED, 0);
                sleep(500);
                mecanumMoveBot(MOVE_SPEED, 0, 0);
                sleep(750);
                mecanumMoveBot(0, 0, 0);
                while (opModeIsActive() && rangeError != 0 && headingError != 0 && yawError != 0) {
                    aprilTagDetectionMovement();
                }
            } else if (x > LEFT_LINE && x < RIGHT_LINE) {
                desiredTagId = 5;
                mecanumMoveBot(MOVE_SPEED, 0, 0);
                sleep(1000);
                mecanumMoveBot(0, 0, 0);
                while (opModeIsActive() && rangeError != 0 && headingError != 0 && yawError != 0) {
                    aprilTagDetectionMovement();
                }
            } else if (x > RIGHT_LINE) {
                desiredTagId = 6;
                mecanumMoveBot(0, -MOVE_SPEED, 0);
                sleep(500);
                mecanumMoveBot(MOVE_SPEED, 0, 0);
                sleep(750);
                mecanumMoveBot(0, 0, 0);
                while (opModeIsActive() && rangeError != 0 && headingError != 0 && yawError != 0) {
                    aprilTagDetectionMovement();
                }
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
            builder.setCamera(webcam1);
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

        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.update();
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(webcam1)
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void aprilTagDetectionMovement() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if ((desiredTagId < 0) || (detection.id == desiredTagId)) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
            detectedTagId = detection.id;
        }

        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            sleep(1000);

            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;

            drive = Range.clip(rangeError * APRIL_TAG_SPEED_GAIN, -APRIL_TAG_MAX_AUTO_SPEED, APRIL_TAG_MAX_AUTO_SPEED);
            turn = Range.clip(headingError * APRIL_TAG_TURN_GAIN, -APRIL_TAG_MAX_AUTO_TURN, APRIL_TAG_MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * APRIL_TAG_STRAFE_GAIN, -APRIL_TAG_MAX_AUTO_STRAFE, APRIL_TAG_MAX_AUTO_STRAFE);

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            aprilTagMoveBot(drive, strafe, turn);
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

    public void aprilTagMoveBot(double x, double y, double yaw) {
        double frontLeftPower =  x - y - yaw;
        double frontRightPower =  x + y + yaw;
        double backLeftPower =  x + y - yaw;
        double backRightPower =  x - y + yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        motorFrontLeft.setPower(frontLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackLeft.setPower(backLeftPower);
        motorBackRight.setPower(backRightPower);
    }

    private void armMovement(double power, int targetPosition) {
        if (targetPosition >= MAX_ARM_POSITION) {
            arm.setTargetPosition((int)MAX_ARM_POSITION);
            arm.setPower(power);
        }
        if (targetPosition <= MIN_ARM_POSITION) {
            arm.setTargetPosition((int) MIN_ARM_POSITION);
            arm.setPower(power);
        } else {
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
        } else {
            lift.setTargetPosition(targetPosition);
            lift.setPower(power);
        }
    }

    private void intakeMovement(double power, double setTime) {
        intake.setPower(power);
        sleep((long) setTime);
        intake.setPower(0);
    }

    private void dumpy_4Movement(double targetPosition) {
        if (targetPosition >= MAX_DUMPY_POSITION) {
            dumpy_4.setPosition(MAX_DUMPY_POSITION);
        } else {
            dumpy_4.setPosition(Math.max(targetPosition, MIN_DUMPY_POSITION));
        }
    }
}
