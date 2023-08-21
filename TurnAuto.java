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
    // Positve yaw is left. Negative yaw is right. Left=L Right=R
    public void Rotate(char direction, double angle, double motorPower){
        imu.resetYaw();
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Back_Right");
        if(direction == 'R'){
            while (yaw >= angle){
                motorFrontLeft.setPower(motorPower);
                motorBackLeft.setPower(motorPower);
                motorFrontRight.setPower(-motorPower);
                motorBackRight.setPower(-motorPower);
            }
        }
        if(direction == 'L'){
           while (yaw <= angle){
                motorFrontLeft.setPower(-motorPower);
                motorBackLeft.setPower(-motorPower);
                motorFrontRight.setPower(motorPower);
                motorBackRight.setPower(motorPower);
            } 
        }
    }
}
