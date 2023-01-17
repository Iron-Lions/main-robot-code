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

enum RGBColor {
    RED, GREEN, BLUE, INVALID
}

@Autonomous(name = "Auto", group = "")

public class Auto extends LinearOpMode {

    private static final double RUN_TIME = 25;
    private static final double MOVE_SPEED = 0.5;
    private static final double DIST_TO_TIME = 3000 / 31.75;
    private static final double SERVO_LOWER = 0.509;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;
    private Servo claw_servo;
    private ColorSensor col_sensor;

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
        
        claw_servo.setPosition(SERVO_LOWER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        // while (opModeIsActive()) {
        //     mecanumMoveBot(MOVE_SPEED, 0, 0);
        //     sleep(5000);
        //     mecanumMoveBot(0, 0, 0);

        //     secondsElapsed = runtime.seconds();
        //     if (secondsElapsed > RUN_TIME) {
        //         break;
        //     }

        //     // Show the elapsed game time and wheel power.
        //     telemetry.addData("secondsElapsed", secondsElapsed);
        //     telemetry.update();
        // }
        
        if (opModeIsActive()) {
            mecanumMoveBot(MOVE_SPEED, 0, 0);
            waitDistance(15.5);
            mecanumMoveBot(0, 0, 0);
            RGBColor coneColor = getRGB();
            telemetry.addData("Color detected", coneColor);
            telemetry.update();
            if (coneColor == RGBColor.BLUE) {
                mecanumMoveBot(MOVE_SPEED, 0, 0);
                waitDistance(36);
                mecanumMoveBot(0, 0, 0);
            }
        }
        
        secondsElapsed = runtime.seconds();
        while (secondsElapsed <= RUN_TIME) {
            sleep(100);
        }
    }
    
    private void waitDistance(double inches) {
        sleep((int) (inches * DIST_TO_TIME));
    }
    
    private RGBColor getRGB() {
        int red, green, blue;
        red = col_sensor.red();
        green = col_sensor.green();
        blue = col_sensor.blue();
        
        if (red >= green && red >= blue) {
            return RGBColor.RED;
        }
        
        if (green >= red && green >= blue) {
            return RGBColor.GREEN;
        }
        
        if (blue >= red && blue >= green) {
            return RGBColor.BLUE;
        }
        
        return RGBColor.INVALID;
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

