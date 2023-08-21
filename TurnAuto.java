package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "TurnAuto", group = "")

public class TurnAuto extends LinearOpMode {
    private IMU imu;
    private double yaw;
    String ran_rotate_method = "no";
    
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
        if(opModeIsActive()){
            Rotate('R',90.0,0.25);
            Rotate('L',180.0,0.25);
            Rotate('R',90.0,0.25);
        }
        
        while (opModeIsActive()) {
            yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.update();
        }
    }
    // F=forwards, B=backwards
    /* public void movement(String direction, double motorPower, double distance) {
      
        if(direction == "F") {
            
        }
        
        if(direction == "B"){
            motorPower = -motorPower;
        }
        while (){
            motorFrontRight.setPower(motorPower);
            motorBackRight.setPower(motorPower);
            motorFrontLeft.setPower(motorPower);
            motorBackLeft.setPower(motorPower);
        }    

        
    }
*/
    // Positive yaw is left. Negative yaw is right. Left=L Right=R
    public void Rotate(char direction, double angle, double motorPower){
        imu.resetYaw();
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Back_Right");
        
        angle = angle-7.5;
        
        if(direction == 'R'){
            while (yaw >= -angle){
                yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                motorFrontLeft.setPower(motorPower);
                motorBackLeft.setPower(motorPower);
                motorFrontRight.setPower(-motorPower);
                motorBackRight.setPower(-motorPower);
                telemetry.addData("yaw", yaw);
                telemetry.update();
            } 
        }
        
        if(direction == 'L'){
            while (yaw <= angle){
                yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                motorFrontLeft.setPower(-motorPower);
                motorBackLeft.setPower(-motorPower);
                motorFrontRight.setPower(motorPower);
                motorBackRight.setPower(motorPower);
                telemetry.addData("yaw", yaw);
                telemetry.update();
            } 
        }
        
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        ran_rotate_method = "yes";
    }
}