package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "moveAuto", group = "")

public class moveAuto extends LinearOpMode {
    private IMU imu;
    private double yaw;

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
        imu.resetYaw();

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Back_Right");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            
            
            
            telemetry.addData("yaw", yaw);
            telemetry.update();
        }
    }

    public void movement(char direction, double motorPower, double distance) {

        if (direction == 'b'){
            distance = -distance;
        }

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double distanceTravelled = 0;
        double numOfTicks = (distance/(96*Math.pi))*537.7;

        motorFrontRight.RunMode.RUN_TO_POSITION(distance);
        motorFrontLeft.RunMode.RUN_TO_POSITION(distance);
        motorBackRight.RunMode.RUN_TO_POSITION(distance);
        motorBackRight.RunMode.RUN_TO_POSITION(distance);

        //distanceTravelled will be calculated with encoders

        /*while (distanceTravelled < distance){
            motorFrontRight.setPower(motorPower);
            motorBackRight.setPower(motorPower);
            motorFrontLeft.setPower(motorPower);
            motorBackLeft.setPower(motorPower);
            distanceTravelled = encoder * 96 * 3.14;
        }*/    
        
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        
    }
}
