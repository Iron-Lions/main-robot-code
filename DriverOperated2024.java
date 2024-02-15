package org.firstinspires.ftc.teamcode;
//Need this to use things from ftc java library
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
@TeleOp(name = "DriverOperated2024", group = "")
public class DriverOperated2024 extends LinearOpMode {
    //These are placeholders!!!!!!!!!!!!!!!!!!!!!!!!
    private static final double MAX_LIFT_POSITION = 4000.0;
    private static final double MIN_LIFT_POSITION = 0.0;
    //private static final double MAX_ARM_POSITION = 750000.0;
    //private static final double MIN_ARM_POSITION = -1000000.0;
    //Min and Max of dumpy (pixel dropper) are scaled between the right most (max) and left most (min) positions
    private static final double MAX_DUMPY_POSITION = 1.0;
    private static final double MIN_DUMPY_POSITION = 0.6;
    //Min and Max of Airplane Lanucher
    private static final double MAX_PLANE_POSITION = 0;
    private static final double MIN_PLANE_POSITION = 0.5;
    //3 Positions used for Backdrop servo
    private static final double INIT_BACKDROP_POSITION = 1;
    private static final double READY_BACKDROP_POSITION = 0.4;
    private static final double FINISH_BACKDROP_POSITION = 0;
    private static final double MAX_PIXEL_RELEASE_POSITION = 0.7;
    private static final double MIN_PIXEL_RELEASE_POSITION = 0.3;
    private static final double MAX_ARM_POSITION = 0;
    private static final double MIN_ARM_POSITION = 0.6;
    private static final double MAX_BUCKET_POSITION = 0.4;
    private static final double MIN_BUCKET_POSITION = 0.52;
    //Factor to multiply by when sensitivity mode is activated
    private static final double SENS_FACTOR = 0.5;
    //declaring motors, servos, and imu
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;
    private DcMotor lift;
    private DcMotor lift4;
    private Servo arm;
    private DcMotor intake;
    private Servo airplane;
    private Servo backdrop;
    private Servo bucket;
    private Servo pixel_release;
    private IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        Gamepad DRIVE_GAMEPAD = gamepad1;
        Gamepad INTAKE_GAMEPAD = gamepad1;
        Gamepad ARM_GAMEPAD = gamepad2;
        Gamepad LIFT_GAMEPAD = gamepad2;
        Gamepad SERVO_GAMEPAD = gamepad2;
        double liftEncoderPosition;
        double liftPower;
        double armPosition = 0.74;
        double bucketPosition = 0.52;
        double pixel_releasePosition = 0.0;
        double armPower;
        double dumpy_4Position;
        double check1 = 0;
        double check2 = 0;
        double intakePower = 1;
        boolean encoderLimitArm = true;
        //Gets information from configuration on driverhub
        motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        motorBackRight = hardwareMap.dcMotor.get("Back_Right");
        lift = hardwareMap.dcMotor.get("lift_main");
        lift4 = hardwareMap.dcMotor.get("lift_mirrored");
        arm = hardwareMap.servo.get("arm");
        intake = hardwareMap.dcMotor.get("intake");
        //dumpy_4 = hardwareMap.servo.get("dumpy_4");
        airplane = hardwareMap.servo.get("airplane");
        backdrop = hardwareMap.servo.get("backdrop");
        bucket = hardwareMap.servo.get("bucket");
        //pixel_dropper = hardwareMap.servo.get("pixel_dropper");
        pixel_release = hardwareMap.servo.get("pixel_release");
        //Allows us to determine position (encoders) and makes motors stop when unpowered
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
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
        arm.setPosition(MIN_ARM_POSITION);
        bucket.setPosition(MIN_BUCKET_POSITION);
        pixel_release.setPosition(MIN_PIXEL_RELEASE_POSITION);
        airplane.setPosition(MIN_PLANE_POSITION);
        backdrop.setPosition(FINISH_BACKDROP_POSITION);
        while (opModeIsActive()) {
            double y = DRIVE_GAMEPAD.left_stick_y; // Remember, this is reversed!
            double x = -DRIVE_GAMEPAD.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -DRIVE_GAMEPAD.right_stick_x;
            liftEncoderPosition = lift.getCurrentPosition();
            liftPower = -LIFT_GAMEPAD.left_stick_y;
            //This code ensures the lift doesn't move too far and damage the robot
            if (liftEncoderPosition <= MIN_LIFT_POSITION) {
                lift.setPower(Math.max(liftPower, 0));
                lift4.setPower(Math.max(liftPower, 0));
            } else if (liftEncoderPosition >= MAX_LIFT_POSITION) {
                lift.setPower(Math.min(liftPower, 0));
                lift4.setPower(Math.min(liftPower, 0));
            }
            else {
                lift.setPower(liftPower);
                lift4.setPower(liftPower);;
            }
            
            if(ARM_GAMEPAD.right_trigger>0.5){
                pixel_release.setPosition(MAX_PIXEL_RELEASE_POSITION);
            }
            else if(ARM_GAMEPAD.left_trigger>0.5){
                pixel_release.setPosition(MIN_PIXEL_RELEASE_POSITION);
            }
            else if(ARM_GAMEPAD.right_bumper){
                arm.setPosition(MAX_ARM_POSITION);
                bucket.setPosition(MAX_BUCKET_POSITION);
            }
            else if(ARM_GAMEPAD.left_bumper){
                arm.setPosition(MIN_ARM_POSITION);
                bucket.setPosition(MIN_BUCKET_POSITION);
            }
            if(INTAKE_GAMEPAD.right_bumper){
                intake.setPower(-intakePower);
            }
            else if(INTAKE_GAMEPAD.left_bumper){
                intake.setPower(intakePower);
            }
            else{
                intake.setPower(0);
            }
            //This fires the airplane if a is pressed
            if(SERVO_GAMEPAD.a){
                airplane.setPosition(MAX_PLANE_POSITION);
            }

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (DRIVE_GAMEPAD.start) {
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
            //code for sensitivity mode (hold)
            if(/*DRIVE_GAMEPAD.left_trigger == 1 ||*/ DRIVE_GAMEPAD.right_trigger == 0){
                frontLeftPower *= SENS_FACTOR;
                backLeftPower *= SENS_FACTOR;
                frontRightPower *= SENS_FACTOR;
                backRightPower *= SENS_FACTOR;
            }
            //Set power to wheels
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            //Telemetry prints data to driverhub
            telemetry.addData("Lift Position", liftEncoderPosition);
            telemetry.addData("Lift Power", liftPower);
            telemetry.addData("Arm Position", armPosition);
            telemetry.addData("bucket Position", bucketPosition);
            telemetry.addData("pixel release", pixel_releasePosition);
            //telemetry.addData("Arm Power", armPower);
            telemetry.addData("check1", check1);
            telemetry.addData("check2", check2);
            //telemetry.addData("dumpy_4 Position", dumpy_4Position);
            telemetry.update();
        }
    }
}
