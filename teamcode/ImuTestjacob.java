package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "ImuTestJacob", group = "")

public class ImuTestJacob extends LinearOpMode {
    private IMU imu;
    private double yaw;
    private double roll;
    private double pitch;

    
    @Override
    public void runOpMode()  throws InterruptedException{
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
            )
        );

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Back_Right");


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
             

            roll = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
            pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
            

            double denominator = 180;

            double roll_power = roll/180;
            double pitch_power = pitch/180;

           /* 
            motorFrontRight.setPower(roll_power);
            motorBackRight.setPower(roll_power);
            motorFrontLeft.setPower(roll_power);
            motorBackLeft.setPower(roll_power);*/
            
            motorFrontRight.setPower(pitch_power);
            motorBackRight.setPower(pitch_power);
            motorFrontLeft.setPower(pitch_power);
            motorBackLeft.setPower(pitch_power);


            telemetry.addData("Roll", roll);
            //telemetry.addData("Pitch", pitch);
            telemetry.update();
        }
    }
    
}
