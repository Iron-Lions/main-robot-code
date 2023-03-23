package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "EncoderTest", group = "")
public class EncoderTest extends LinearOpMode {
    private DcMotor testMotor;

    @Override
    public void runOpMode() {
        testMotor = hardwareMap.dcMotor.get("BR");

        testMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        int encoderPosition;
        // double power;
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setPower(1.0);
        testMotor.setTargetPosition(5000);
        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {
            // power = -gamepad1.left_stick_y;
            // testMotor.setPower(power);
            encoderPosition = testMotor.getCurrentPosition();

            // telemetry.addData("power", power);
            telemetry.addData("position", encoderPosition);
            telemetry.addData("target position", testMotor.getTargetPosition());
            telemetry.update();
        }
    }
}
