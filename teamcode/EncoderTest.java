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
        testMotor = hardwareMap.dcMotor.get("motor");

        testMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        int encoderPosition;
        double power;
        while (opModeIsActive()) {
            power = -gamepad1.left_stick_y;
            testMotor.setPower(power);
            encoderPosition = testMotor.getCurrentPosition();

            telemetry.add("power", power);
            telemetry.add("position", encoderPosition);
            telemetry.update();
        }
    }
}
