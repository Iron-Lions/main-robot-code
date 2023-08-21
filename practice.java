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

    @Override
    public void runOpMode() {
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Back_Right");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
            )
        );
        
        waitForStart();
        while (opModeIsActive()) {
            roll = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
            pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);

            double denominator = 180;
            //double denominator = Math.max(Math.abs(roll) + Math.abs(pitch), 1);

            /*if (roll>0){
                motorFrontRight.setPower((roll)/denominator);
                motorBackRight.setPower((roll)/denominator);
            }
            if (roll<0){
                motorFrontLeft.setPower((roll)/denominator);
                motorBackLeft.setPower((roll)/denominator);
            }*/
            
            if (pitch>0){
                motorFrontRight.setPower((pitch)/denominator);
                motorFrontLeft.setPower((pitch)/denominator);
            }
            if (pitch<0){
                motorBackRight.setPower((pitch)/denominator);
                motorBackLeft.setPower((pitch)/denominator);
            }
            
            telemetry.addData("Roll (Degrees)", roll);
            telemetry.addData("Pitch (Degrees)", pitch);
            telemetry.update();
        }
    }
}