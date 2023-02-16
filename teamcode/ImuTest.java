package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "ImuTest", group = "")

public class ImuTest extends LinearOpMode {
    private IMU imu;
    double yaw;
    
    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
            )
        );
        
        waitForStart();
        while (opModeIsActive()) {
            yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            
            telemetry.addData("Yaw (Degrees)", yaw);
            telemetry.update();
        }
    }
}