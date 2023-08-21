package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ImuTestAlden", group = "")

public class ImuTestAlden extends LinearOpMode {
    private IMU imu;
    private double roll;
    private double pitch;
    private double frontRightPower=0;
    private double frontLeftPower=0;
    private double backRightPower=0;
    private double backLeftPower=0;
    
    @Override
    public void runOpMode() {
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Back_Right");
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
            )
        );
        
        imu.resetYaw();
        
        waitForStart();
        while (opModeIsActive()) {
            roll = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
            pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
            
            double denominator = 180;
            //double denominator = Math.max(Math.abs(roll) + Math.abs(pitch), 1.0);
            
            if (roll > 0){
                frontRightPower += roll/denominator;
                backRightPower += roll/denominator;
            }
            
            if (roll < 0){
                frontLeftPower += roll/denominator;
                backLeftPower += roll/denominator;
            }
            
            if (pitch > 0){
                frontRightPower += pitch/denominator;
                frontLeftPower += pitch/denominator;
            }
            
            if (pitch > 0){
                backRightPower += pitch/denominator;
                backLeftPower += pitch/denominator;
            }
            
            motorFrontRight.setPower(frontRightPower);
            motorFrontLeft.setPower(frontLeftPower);
            motorBackRight.setPower(backRightPower);
            motorBackLeft.setPower(backLeftPower);
            
            frontRightPower = 0;
            frontLeftPower = 0;
            backRightPower = 0;
            backLeftPower = 0;
            
            telemetry.addData("Roll (Degrees)", roll);
            telemetry.addData("Pitch (Degrees)", pitch);
            telemetry.update();
        }

    }
}