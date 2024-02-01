package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "newAuto", group = "")

public class newAuto extends LinearOpMode {
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
        if(opModeIsActive()){
            Rotate('R',15,0.5);
            Rotate('L',15,0.5);
        }
        
        while (opModeIsActive()) {
            yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("Encoder Value", motorFrontRight.getCurrentPosition());
            telemetry.addData("Target Value", motorFrontRight.getTargetPosition());
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
        
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        
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
    }
    public void movement(char direction, double motorPower, double distance) {
        
        int distanceTravelled = 0;
        
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Back_Right");
        
        
        if (direction == 'b'){
            distance = -distance;
        }
        
        int numOfTicks = (int)Math.round((distance/(96*Math.PI))*537.7);
        
        telemetry.addData("Ticks", numOfTicks);
        telemetry.update();
        
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        telemetry.addData("Reset Encoder", "True");
        telemetry.update();
        
        motorFrontRight.setTargetPosition(numOfTicks);
        motorFrontLeft.setTargetPosition(numOfTicks);
        motorBackRight.setTargetPosition(numOfTicks);
        motorBackLeft.setTargetPosition(numOfTicks);
        
        telemetry.addData("Set Target Position", "True");
        telemetry.addData("Ticks", numOfTicks);
        telemetry.update();
        
        motorFrontRight.setPower(motorPower);
        motorBackRight.setPower(motorPower);
        motorFrontLeft.setPower(motorPower);
        motorBackLeft.setPower(motorPower);
        
        telemetry.addData("Set Motor Power", "True");
        telemetry.addData("Target Value", motorFrontRight.getTargetPosition());
        telemetry.update();
        
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        telemetry.addData("Encoder Value", motorFrontRight.getCurrentPosition());
        telemetry.update();
        
        while(opModeIsActive() && motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorBackLeft.isBusy() && motorBackRight.isBusy()) {
            idle();
        }
        
        //distanceTravelled will be calculated with encoders
        
        
        /*while (distanceTravelled < numOfTicks){
            motorFrontRight.setPower(motorPower);
            motorBackRight.setPower(motorPower);
            motorFrontLeft.setPower(motorPower);
            motorBackLeft.setPower(motorPower);
            int encoder = motorFrontRight.getCurrentPosition();
            distanceTravelled = (int)(encoder * 96 * 3.14);
            telemetry.addData("Distance Travelled", distanceTravelled);
            telemetry.addData("encoder", encoder);
            telemetry.addData("Ticks", numOfTicks);
            telemetry.update();
        }*/
    }
}
