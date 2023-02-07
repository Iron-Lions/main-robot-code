package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "Auto (New)", group = "")

public class Auto extends LinearOpMode {

    private static final double RUN_TIME = 25;
    private static final double MOVE_SPEED = 0.5;
    private static final double FB_DIST_TO_TIME = 3000 / 31.75;
    private static final double LR_DIST_TO_TIME = 3000 / 17.5;
    private static final double SERVO_LOWER = 0.509;
    private static final double SERVO_UPPER = 0.795;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;
    private Servo claw_servo;
    private ColorSensor col_sensor;

    private double x = 0;
    private double y = 0;

    @Override
    public void runOpMode() {
        double secondsElapsed;

        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        claw_servo = hardwareMap.servo.get("claw_servo");
        col_sensor = hardwareMap.colorSensor.get("color_sensor");

        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        claw_servo.setPosition(SERVO_UPPER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            moveDistanceFB(-MOVE_SPEED, 19);
            sleep(1000);
            int zone = getZone();
            telemetry.addData("Cone zone detected", zone);
            telemetry.update();
            if (zone == 2) {
                moveDistanceFB(-MOVE_SPEED, 26);
            }
            if (zone == 3) {
                moveDistanceFB(-MOVE_SPEED, 3);
                moveDistanceLR(-MOVE_SPEED, 21);
            }
            if (zone == 1) {
                moveDistanceFB(-MOVE_SPEED, 3);
                moveDistanceLR(MOVE_SPEED, 21);
            }

        }

    }

    private void waitDistanceFB(double inches) {
        sleep((int) (inches * FB_DIST_TO_TIME));
    }

    private void waitDistanceLR(double inches) {
        sleep((int) (inches * LR_DIST_TO_TIME));
    }

    private void moveDistanceFB(double speed, double inches) {
        mecanumMoveBot(speed, 0, 0);
        waitDistanceFB(inches);
        mecanumMoveBot(0, 0, 0);
    }

    private void moveDistanceLR(double speed, double inches) {
        mecanumMoveBot(0, speed, 0);
        waitDistanceLR(inches);
        mecanumMoveBot(0, 0, 0);
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
