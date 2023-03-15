package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ArcadeDrive", group = "")
public class ArcadeDrive extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    public static final double SPEED_SCALE = 0.25;
    public static final DcMotor.ZeroPowerBehavior START_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.dcMotor.get("L");
        rightMotor = hardwareMap.dcMotor.get("R");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(START_BEHAVIOR);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(START_BEHAVIOR);

        double fb_movement, rotation;

        waitForStart();
        while (opModeIsActive()) {
            // Read inputs from controller
            fb_movement = -gamepad1.left_stick_y;
            rotation = gamepad1.right_stick_x;

            // Set the drivetrain power
            arcadeMoveBot(fb_movement, rotation);

            // Print telemetry data
            telemetry.addData("fb_movement", fb_movement);
            telemetry.addData("rotation", rotation);
            telemetry.update();
        }
    }

    public void arcadeMoveBot(double fb_translation, double rotation) {
        double left_motor_power = (fb_translation + rotation) * SPEED_SCALE;
        double right_motor_power = (fb_translation - rotation) * SPEED_SCALE;
        leftMotor.setPower(left_motor_power);
        rightMotor.setPower(right_motor_power);
    }
}
