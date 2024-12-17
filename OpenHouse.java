package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="OpenHouse", group="TeleOP")
public class OpenHouse extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Back_Right");
        
        DcMotor lift = hardwareMap.dcMotor.get("Lift");
        Servo shoulderL = hardwareMap.servo.get("Left_Shoulder");
        Servo shoulderR = hardwareMap.servo.get("Right_Shoulder");
        Servo wrist = hardwareMap.servo.get("Wrist");
        //Servo fingers = /**/;
        
        //NOT FINAL!!!
        double shoulderMinValue = 50;
        double shoulderMaxValue = 100;
        
        double wristMinValue = 150;
        double wristMaxValue = 100;
        
        double shoulderPos = 0;

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double shoulderMotion = gamepad2.left_stick_y;
            shoulderPos += (shoulderMotion);
            double elbowMotion = gamepad2.right_stick_y;
            
            if(gamepad2.a) {
                wrist.setPosition(wristMaxValue);
            }
            if(gamepad2.b) {
                wrist.setPosition(wristMinValue);
            }
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            
            shoulderL.setPosition(shoulderPos);
            shoulderR.setPosition(-shoulderPos);
            
            telemetry.addData("Left shoulder postion", shoulderL.getPosition());
            telemetry.addData("Right shoulder postion", shoulderR.getPosition());
            telemetry.addData("Wrist postion", wrist.getPosition());
            telemetry.update();
        }
    }
}