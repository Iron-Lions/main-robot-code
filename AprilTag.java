package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "AprilTag", group = "")
public class AprilTag extends LinearOpMode {

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor lift;
    private DcMotor arm;
    private DcMotor intake;
    private Servo dumpy_4;
    private static final double MAX_LIFT_POSITION = 0;
    private static final double MIN_LIFT_POSITION = -3100.0;
    private static final double MAX_ARM_POSITION = 750.0;
    private static final double MIN_ARM_POSITION = -10000.0;
    private static final double MAX_DUMPY_POSITION = 1.0;
    private static final double MIN_DUMPY_POSITION = 0.5;
    private static final boolean USE_WEBCAM = true;
    private WebcamName webcam1;
    private static final double DESIRED_DISTANCE = 12.0;
    private static final double APRIL_TAG_SPEED_GAIN = 0.02;
    private static final double APRIL_TAG_STRAFE_GAIN = 0.015;
    private static final double APRIL_TAG_TURN_GAIN = 0.01;
    private static final double APRIL_TAG_MAX_AUTO_SPEED = 0.5;
    private static final double APRIL_TAG_MAX_AUTO_STRAFE = 0.5;
    private static final double APRIL_TAG_MAX_AUTO_TURN = 0.3;
    private int desiredTagId;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private int detectedTagId;
    private double drive = -1;
    private double strafe = -1;
    private double turn = -1;
    double rangeError = -1;
    private double headingError = -1;
    private double yawError = -1;
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
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        initAprilTag();
        setManualExposure(6, 250); // Use low exposure time to reduce motion blur
        waitForStart();


        if (opModeIsActive()) {
            desiredTagId = 4;
            while (opModeIsActive() && ((detectedTagId != desiredTagId) || (detectedTagId == desiredTagId && (Math.abs(drive) > 0.1 || Math.abs(turn) > 0.1 || Math.abs(strafe) > 0.1)))) {
                processAprilTag();
            }
        }
        visionPortal.close();
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

    private void processAprilTag() {
        boolean targetFound = false;
        desiredTag = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if ((desiredTagId < 0) || (detection.id == desiredTagId)) {
                    desiredTag = detection;
                    detectedTagId = detection.id;
                    targetFound = true;
                    break;
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
            telemetry.update();
        }
        
        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            telemetry.update();

            rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            headingError = desiredTag.ftcPose.bearing;
            yawError = desiredTag.ftcPose.yaw;

            drive = Range.clip(rangeError * APRIL_TAG_SPEED_GAIN, -APRIL_TAG_MAX_AUTO_SPEED, APRIL_TAG_MAX_AUTO_SPEED);
            turn = Range.clip(headingError * APRIL_TAG_TURN_GAIN, -APRIL_TAG_MAX_AUTO_TURN, APRIL_TAG_MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * APRIL_TAG_STRAFE_GAIN, -APRIL_TAG_MAX_AUTO_STRAFE, APRIL_TAG_MAX_AUTO_STRAFE);

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }

        telemetry.update();
        aprilTagMoveBot(drive, strafe, turn);
        sleep(10); // Wait to update
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

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
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
