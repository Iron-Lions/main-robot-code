package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Controls:
 * gamepad1 left_stick_y - left motor
 * gamepad1 right_stick_y - right motor
 * gamepad1 right_trigger - move arm up
 * gamepad1 left_trigger - move arm down
 */
@TeleOp(name = "Jarvis", group = "")
public class Jarvis extends LinearOpMode {
  private DcMotor leftMotor;
  private DcMotor rightMotor;
  private DcMotor armMotor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double left_motor_power, right_motor_power, arm_motor_power;

    leftMotor = hardwareMap.dcMotor.get("L");
    rightMotor = hardwareMap.dcMotor.get("R");
    armMotor = hardwareMap.dcMotor.get("arm");

    leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();
    while (opModeIsActive()) {
      left_motor_power = gamepad1.left_stick_y;
      right_motor_power = gamepad1.right_stick_y;
      leftMotor.setPower(left_motor_power);
      rightMotor.setPower(right_motor_power);

      arm_motor_power = gamepad1.right_trigger - gamepad1.left_trigger;
      armMotor.setPower(arm_motor_power);

      telemetry.addData("left_motor_power", left_motor_power);
      telemetry.addData("right_motor_power", right_motor_power);
      telemetry.addData("arm_motor_power", arm_motor_power);
      telemetry.update();
    }
  }
}
