package org.firstinspires.ftc.teamcode;

import java.lang.Math;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name = "Auto (Score on Left)", group = "")

public class AutoCoords extends LinearOpMode {

    private static final double RUN_TIME = 25;
    private static final double MOVE_SPEED = 0.5;
    private static final double ROTATE_SPEED = 0.4;
    private static final double FB_DIST_TO_TIME = 3000 / 31.75;
    private static final double LR_DIST_TO_TIME = 3000 / 17.5;
    private static final double SERVO_LOWER = 0.509; // Claw open
    private static final double SERVO_UPPER = 0.795; // Claw closed
    private static final int MOVE_WAIT = 500;
    private ElapsedTime runtime = new ElapsedTime();
    private int arm4_initial;

    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;
    private DcMotor arm4;
    private DcMotor arm4_r;
    private Servo claw_servo;
    private ColorSensor col_sensor;
    private IMU imu;
    private double yaw;

    private double current_x = 0;
    private double current_y = 0;

    private void initialize_hardware() {
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        arm4 = hardwareMap.dcMotor.get("arm4");
        arm4_r = hardwareMap.dcMotor.get("arm4_r");
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
    
        arm4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm4_r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        claw_servo = hardwareMap.servo.get("claw_servo");
        col_sensor = hardwareMap.colorSensor.get("color_sensor");
        
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
            )
        );

        claw_servo.setPosition(SERVO_LOWER);
        arm4_initial = arm4.getCurrentPosition();
    }

    @Override
    public void runOpMode() {  
        initialize_hardware();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        claw_servo.setPosition(SERVO_UPPER);

        if (opModeIsActive()) {
            sleep(1000);
            arm4.setPower(1);
            arm4_r.setPower(-1);
            while(arm4.getCurrentPosition() < (arm4_initial + 2000)) {
                sleep(50);
            }
            arm4.setPower(0);
            arm4_r.setPower(0);

            sleep(1000);

            goToFBLR(0, 20);
            sleep(1000);
            int zone = getZone();
            telemetry.addData("Cone zone detected", zone);
            telemetry.update();
            if (zone == 2) {
                goToFBLR(0, 50);
                goToFBLR(0, 40);
            }
            if (zone == 3) {
                goToFBLR(0, 30);
                goToFBLR(33, 23);
            }
            if (zone == 1) {
                goToFBLR(0, 30);
                goToFBLR(-35, 23);
            }
            sleep(1000);
            arm4.setPower(-0.5);
            arm4_r.setPower(0.5);
            while(arm4.getCurrentPosition() > arm4_initial) {
                sleep(0);
            }
            arm4.setPower(0);
            arm4_r.setPower(0);
        }
    }

    private void waitDistanceFB(double inches) {
        sleep((int) (inches * FB_DIST_TO_TIME));
    }

    private void waitDistanceLR(double inches) {
        sleep((int) (inches * LR_DIST_TO_TIME));
    }

    private void moveDistanceFB(double inches) {
        current_y += inches;
        if (inches < 0) {
            mecanumMoveBot(-MOVE_SPEED, 0, 0);
        } else {
            mecanumMoveBot(MOVE_SPEED, 0, 0);
        }
        waitDistanceFB(Math.abs(inches));
        mecanumMoveBot(0, 0, 0);
    }

    private void moveDistanceLR(double inches) {
        current_x += inches;
        if (inches < 0) {
            mecanumMoveBot(0, -MOVE_SPEED, 0);
        } else {
            mecanumMoveBot(0, MOVE_SPEED, 0);
        }
        waitDistanceFB(Math.abs(inches));
        mecanumMoveBot(0, 0, 0);
    }

    private void goToFBLR(double target_x, double target_y) {
        double delta_x = target_x - current_x;
        double delta_y = target_y - current_y;

        moveDistanceFB(delta_y);
        moveDistanceLR(delta_x);
        sleep(MOVE_WAIT);
    }

    private int getZone() {
        int red = col_sensor.red();
        int green = col_sensor.green();
        int blue = col_sensor.blue();

        if (blue >= red && blue >= green) {
            return 2;
        }

        if (red >= green && red >= blue) {
            return 1;
        }

        if (green >= red && green >= blue) {
            return 3;
        }

        return 0;
    }

    /**
     * Move robot using Mechanum wheels, given translation and rotation.
     */
    private void mecanumMoveBot(double FB_translation, double LR_translation, double rotation) {
        double FL_power = FB_translation;
        double BL_power = FB_translation;
        double FR_power = FB_translation;
        double BR_power = FB_translation;
        FL_power += LR_translation;
        FR_power -= LR_translation;
        BL_power -= LR_translation;
        BR_power += LR_translation;
        FL_power += rotation;
        FR_power -= rotation;
        BL_power += rotation;
        BR_power -= rotation;

        FL.setPower(FL_power);
        BL.setPower(BL_power);
        FR.setPower(FR_power);
        BR.setPower(BR_power);
    }
}
