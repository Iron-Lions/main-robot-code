package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Auto", group = "")

public class Auto extends LinearOpMode {

    private static final double RUN_TIME = 25;
    private static final double MOVE_SPEED = 0.5;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;

    @Override
    public void runOpMode() {
        double secondsElapsed;

        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");

        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            mecanumMoveBot(MOVE_SPEED, 0, 0);
            while (runtime.seconds() < 5) {
            }
            mecanumMoveBot(0, 0, 0);

            secondsElapsed = runtime.seconds();
            if (secondsElapsed > RUN_TIME) {
                break;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("secondsElapsed", secondsElapsed);
            telemetry.update();
        }
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
