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
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double secondsElapsed;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            secondsElapsed = runtime.seconds();
            if (secondsElapsed > RUN_TIME) {
                break;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("secondsElapsed", secondsElapsed);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
