package org.firstinspires.ftc.teamcode;
//Need this to use things from ftc java library
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    private static final double MAX_LIFT_POSITION = 0;
    private static final double MIN_LIFT_POSITION = -3100.0;
    private static final double MAX_ARM_POSITION = 750.0;
    private static final double MIN_ARM_POSITION = -10000.0;
    //Min and Max of dumpy (servo) are scaled between the right most (max) and left most (min) positions
    private static final double MAX_DUMPY_POSITION = 1.0;
    private static final double MIN_DUMPY_POSITION = 0.5;
    
    //declaring motors, servos, and imu
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;
    private DcMotor lift;
    private DcMotor arm;
    private DcMotor intake;
    private Servo dumpy_4;
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
        double armEncoderPosition;
        double armPower;
        double dumpy_4Position;
        double check1 = 0;
        double check2 = 0;
        boolean encoderLimitArm = true;
        
        //Gets information from configuration on driverhub
        motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        motorBackRight = hardwareMap.dcMotor.get("Back_Right");
        intake = hardwareMap.dcMotor.get("intake");
        lift = hardwareMap.dcMotor.get("lift");
        arm = hardwareMap.dcMotor.get("arm");
        dumpy_4 = hardwareMap.servo.get("dumpy_4");
        
        //Allows us to determine position (encoders) and makes motors stop when unpowered
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

            liftEncoderPosition = lift.getCurrentPosition();
            liftPower = LIFT_GAMEPAD.left_stick_y;
            
            //This code ensures the lift doesn't move too far and damage the robot
            if (liftEncoderPosition <= MIN_LIFT_POSITION) {
                lift.setPower(Math.max(liftPower, 0));
            }
            else if (liftEncoderPosition >= MAX_LIFT_POSITION) {
                lift.setPower(Math.min(liftPower, 0));
            }
            else {
                lift.setPower(liftPower);
            }

            armEncoderPosition = arm.getCurrentPosition();
            armPower = -ARM_GAMEPAD.right_stick_y*0.25;
            
            //This code ensures the arm doesn't move too far and damage the robot
            if (armEncoderPosition <= MIN_ARM_POSITION) {
                arm.setPower(Math.max(armPower, 0));
            }
            else if (armEncoderPosition >= MAX_ARM_POSITION) {
                arm.setPower(Math.min(armPower, 0));
            }
            else if(liftEncoderPosition<=-2500){
                arm.setPower(armPower);
            }

            //You need to hold the bumpers to make it work
            if(INTAKE_GAMEPAD.right_bumper){
                check1 = -0.75;
            }
            else {
                check1 = 0;
            }

            if(INTAKE_GAMEPAD.left_bumper){
                check2 = 0.75;
            }
            else {
                check2 = 0;
            }
            intake.setPower(check1 + check2);
            
            //This is to print out telemetry later
            dumpy_4Position = dumpy_4.getPosition();
            
            //Servo controls-can only move dumpy_4 to two locations
            if(SERVO_GAMEPAD.left_trigger > 0.5){
                dumpy_4.setPosition(MAX_DUMPY_POSITION);
            }

            if(SERVO_GAMEPAD.right_trigger > 0.5){
                dumpy_4.setPosition(MIN_DUMPY_POSITION);
            }

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (DRIVE_GAMEPAD.options) {
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
            
            //Set power to wheels
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            
            //Telemetry prints data to driverhub
            telemetry.addData("Lift Position", liftEncoderPosition);
            telemetry.addData("Lift Power", liftPower);
            telemetry.addData("Arm Position", armEncoderPosition);
            telemetry.addData("Arm Power", armPower);
            telemetry.addData("check1", check1);
            telemetry.addData("check2", check2);
            telemetry.addData("dumpy_4 Position", dumpy_4Position);
            telemetry.update();
        }
    }
}