package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto_Test", group = "")

public class Auto_Test extends LinearOpMode {
    private IMU imu;
    private double yaw;
    private ElapsedTime runtime = new ElapsedTime();

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
        runtime.reset();
        movement('b',1.0,50000);
        
        while (opModeIsActive()) {
            yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
            telemetry.addData("yaw", yaw);
            telemetry.update();
        }
    }

    public void movement(char direction, double motorPower, double distance) {
        
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Back_Right");
        
        
        
        if (direction == 'b'){
            distance = -distance;
        }


        double distanceTravelled = 0;
        int numOfTicks = (int)Math.round((distance/(96*Math.PI))*537.7);
        
        
        motorFrontRight.setTargetPosition(numOfTicks);
        motorFrontLeft.setTargetPosition(numOfTicks);
        motorBackRight.setTargetPosition(numOfTicks);
        motorBackLeft.setTargetPosition(numOfTicks);
        
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //distanceTravelled will be calculated with encoders

        /*while (distanceTravelled < distance){
            motorFrontRight.setPower(motorPower);
            motorBackRight.setPower(motorPower);
            motorFrontLeft.setPower(motorPower);
            motorBackLeft.setPower(motorPower);
            distanceTravelled = encoder * 96 * 3.14;
        } */   
        
        motorFrontRight.setPower(motorPower);
        motorBackRight.setPower(motorPower);
        motorFrontLeft.setPower(motorPower);
        motorBackLeft.setPower(motorPower);
        
    }
}
