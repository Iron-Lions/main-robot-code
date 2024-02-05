package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="MotorTest", group="")
public class MotorTest extends LinearOpMode{

    // todo: write your code here
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontRight;
    private IMU imu;
    private static final double MOVE_SPEED = 0.25;
    private double yaw;

    public void runOpMode() throws InterruptedException{
        motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        motorBackRight = hardwareMap.dcMotor.get("Back_Right");
        motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        imu.resetYaw();

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
    
        
        if (opModeIsActive()) {
                mecanumMoveBotEncoders(MOVE_SPEED, 0, 0,.65);
                Rotate('L', 90, MOVE_SPEED);
                mecanumMoveBotEncoders(MOVE_SPEED, 0, 0, .1);
                //*drop pixel*
                mecanumMoveBotEncoders(-MOVE_SPEED, 0, 0,.1);
                Rotate('R', 180, MOVE_SPEED);
                mecanumMoveBotEncoders(MOVE_SPEED, 0, 0, .65);
                //*flip forward backdrop servo*
                //Align with left april tag
                mecanumMoveBotEncoders(.1, 0, 0,.3);
                mecanumMoveBotEncoders(-MOVE_SPEED, 0, 0, .1);
                //*flip down backdrop servo*
                Rotate('R', 90, MOVE_SPEED);
                mecanumMoveBotEncoders(MOVE_SPEED, 0, 0, .3);
                //**End of program*
        }

    }
    private void mecanumMoveBotEncoders(double FB_translation, double LR_translation, double rotation, double distance) {
        // Calculate individual motor powers
        FB_translation = -FB_translation;
        double FL_power = FB_translation + LR_translation + rotation;
        double BL_power = FB_translation - LR_translation + rotation;
        double FR_power = FB_translation - LR_translation - rotation;
        double BR_power = FB_translation + LR_translation - rotation;

        int FL_direction = 0;
        int BL_direction = 0;
        int FR_direction = 0;
        int BR_direction = 0;

        distance = distance * 1000; //meters to mm

        distance = distance * 0.97035; // Tune ratio

        double numberOfTicks = (distance/(96*Math.PI))*537.7;


        if (FL_power > 0) {
            FL_direction = 1;
        }
        else if (FL_power < 0) {
            FL_direction = -1;
        }
        if (BL_power > 0) {
            BL_direction = 1;
        }
        else if (BL_power < 0) {
            BL_direction = -1;
        }
        if (FR_power > 0) {
            FR_direction = 1;
        }
        else if (FR_power < 0) {
            FR_direction = -1;
        }
        if (BR_power > 0) {
            BR_direction = 1;
        }
        else if (BR_power < 0) {
            BR_direction = -1;
        }

        // Set target encoder positions
        int BLTargetPosition = motorBackLeft.getCurrentPosition() + (int) (numberOfTicks * (BL_direction));
        int FLTargetPosition = motorFrontLeft.getCurrentPosition() + (int) (numberOfTicks * (FL_direction));
        int FRTargetPosition = motorFrontRight.getCurrentPosition() + (int) (numberOfTicks * (FR_direction));
        int BRTargetPosition = motorBackRight.getCurrentPosition() + (int) (numberOfTicks * (BR_direction));

        // Set target positions for encoders
        motorFrontLeft.setTargetPosition(FLTargetPosition);
        motorBackLeft.setTargetPosition(BLTargetPosition);
        motorFrontRight.setTargetPosition(FRTargetPosition);
        motorBackRight.setTargetPosition(BRTargetPosition);

        // Set motor power and run to position
        motorFrontLeft.setPower(FL_power);
        motorBackLeft.setPower(BL_power);
        motorFrontRight.setPower(FR_power);
        motorBackRight.setPower(BR_power);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait until all motors reach their target position
        while (opModeIsActive() && motorFrontLeft.isBusy() && motorBackRight.isBusy() && motorBackLeft.isBusy() && motorFrontRight.isBusy()) {
            telemetry.addData("FrontRight Position", motorFrontRight.getCurrentPosition());
            telemetry.addData("BackLeft Position", motorBackLeft.getCurrentPosition());
            telemetry.addData("FrontLeft Position", motorFrontLeft.getCurrentPosition());
            telemetry.addData("BackRight Position", motorBackRight.getCurrentPosition());
            telemetry.addData("FrontRight Target", motorFrontRight.getTargetPosition());
            telemetry.addData("BackLeft Target", motorBackLeft.getTargetPosition());
            telemetry.addData("FrontLeft Target", motorFrontLeft.getTargetPosition());
            telemetry.addData("BackRight Target", motorBackRight.getTargetPosition());
            telemetry.update();
        }

        // Stop motors
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors back to using encoders
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
    }

    public void Rotate(char direction, double angle, double motorPower){
        imu.resetYaw();
        yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
            DcMotor motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
            DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
            DcMotor motorBackRight = hardwareMap.dcMotor.get("Back_Right");

            motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
            motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

            angle = (angle - 7.5) * 1; // Tuning 1 if needed

            if (direction == 'R') {
                while (yaw >= -angle) {
                    yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    motorFrontLeft.setPower(-motorPower);
                    motorBackLeft.setPower(-motorPower);
                    motorFrontRight.setPower(motorPower);
                    motorBackRight.setPower(motorPower);
                    telemetry.addData("yaw", yaw);
                    telemetry.update();
                }
            }

            if (direction == 'L') {
                while (yaw <= angle) {
                    yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    motorFrontLeft.setPower(motorPower);
                    motorBackLeft.setPower(motorPower);
                    motorFrontRight.setPower(-motorPower);
                    motorBackRight.setPower(-motorPower);
                    telemetry.addData("yaw", yaw);
                    telemetry.update();
                }
            }

            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(0);
            telemetry.addData("Encoder Value", motorFrontRight.getCurrentPosition());
            telemetry.addData("Target Value", motorFrontRight.getTargetPosition());
            telemetry.update();
    }
}
