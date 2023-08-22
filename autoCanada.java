package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="AutoCanada")
public class AutoCanada extends LinearOpMode {
    
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;
    
    private int motorFrontLeftPos;
    private int motorFrontRightPos;
    private int motorBackLeftPos;
    private int motorBackRightPos;
    
    @Override
    public void runOpMode() {
        motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        motorBackRight = hardwareMap.dcMotor.get("Back_Right");
        
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        waitForStart();
        
        drive(1000, 1000, 0.1);
    }

    private void drive(int leftTarget, int rightTarget, double speed){
        motorFrontLeftPos += leftTarget;
        motorFrontRightPos += rightTarget;
        motorBackLeftPos += leftTarget;
        motorBackRightPos += rightTarget;
        
        motorFrontRight.setTargetPosition(motorFrontRightPos);
        motorFrontLeft.setTargetPosition(motorFrontLeftPos);
        motorBackRight.setTargetPosition(motorBackRightPos);
        motorBackLeft.setTargetPosition(motorBackLeftPos);
        
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        motorFrontRight.setPower(speed);
        motorFrontLeft.setPower(speed);
        motorBackRight.setPower(speed);
        motorBackLeft.setPower(speed);
        
        while(opModeIsActive() && motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorBackLeft.isBusy() && motorBackRight.isBusy()) {
            telemetry.addData("Encoder Value", motorFrontRight.getCurrentPosition());
            telemetry.addData("Target Value", motorFrontRight.getTargetPosition());
            telemetry.update();
        }
    }
    
}