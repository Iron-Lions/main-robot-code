package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@TeleOp
public class GoBuilda_Mecanum extends LinearOpMode {
    private IMU imu;
    private DcMotor arm;
    double armEncoderPosition;
    double armPower;

    private static final double MAX_ARM_POSITION = 0.0;
    private static final double MIN_ARM_POSITION = 0.0;



    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        Gamepad DRIVE_GAMEPAD = gamepad1;
        Gamepad ARM_GAMEPAD = gamepad2;
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Back_Right");
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = DRIVE_GAMEPAD.left_stick_y; // Remember, this is reversed!
            double x = -DRIVE_GAMEPAD.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = DRIVE_GAMEPAD.right_stick_x;

            armEncoderPosition = arm.getCurrentPosition();
            armPower = ARM_GAMEPAD.left_stick_y;
            arm.setPower(armPower);

            if (armEncoderPosition < MIN_ARM_POSITION || armEncoderPosition > MAX_ARM_POSITION) {
                arm.setPower(0);
            }

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }


            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            telemetry.addData("Arm Position: ", armEncoderPosition);
        }
    }
}
