package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Controls:
 * gamepad1 left_stick - translational motion
 * gamepad1 right_stick_x - rotational motion
 * gamepad1 right_trigger - move arm up
 * gamepad1 left_trigger - move arm down
 * gamepad1 right_bumper - open claw
 * gamepad1 left_bumper - close claw
 */

@TeleOp(name = "MecanumWheels (Blocks to Java)", group = "")
public class MecanumWheels extends LinearOpMode {
  public static final double REAR_RATIO = 3.0 / 8.0;

  private DcMotor FL;
  private DcMotor BL;
  private DcMotor FR;
  private DcMotor BR; 
  private DcMotor arm4;
  private Servo claw_servo;
  

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double FB_translation;
    double LR_translation;
    double rotation;
    double servo_spin;
    double arm_power;
    double claw_position = 0;


    FL = hardwareMap.dcMotor.get("FL");
    BL = hardwareMap.dcMotor.get("BL");
    FR = hardwareMap.dcMotor.get("FR");
    BR = hardwareMap.dcMotor.get("BR");
    arm4 = hardwareMap.dcMotor.get("arm4");
    claw_servo = hardwareMap.servo.get("claw_servo");

    // Put initialization blocks here.
    FR.setDirection(DcMotorSimple.Direction.FORWARD);
    BR.setDirection(DcMotorSimple.Direction.FORWARD);
    FR.setDirection(DcMotorSimple.Direction.REVERSE);
    BR.setDirection(DcMotorSimple.Direction.REVERSE);
    
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        FB_translation = -gamepad1.left_stick_y;
        LR_translation = gamepad1.left_stick_x;
        rotation = gamepad1.right_stick_x;
        
        if (gamepad1.right_trigger > 0.5) {
          arm_power = 0.5;
        } else if (gamepad1.left_trigger > 0.5) {
          arm_power = -0.5;
        } else {
          arm_power = 0;
        }

        arm4.setPower(arm_power);

        if (gamepad1.right_bumper){
          servo_spin = 0.5;
        } else if (gamepad1.left_bumper){
          servo_spin = -0.5; 
        } else {
          servo_spin = 0.0;
        }
        
        mecanumMoveBot(FB_translation, LR_translation, rotation);
        
        claw_position += servo_spin / 50;
        claw_position = Math.min(Math.max(claw_position, 0), 1);
        claw_servo.setPosition(claw_position);
        
        telemetry.addData("FB_translation", FB_translation);
        telemetry.addData("LR_translation", LR_translation);
        telemetry.addData("rotation", rotation);
        telemetry.addData("Servo_rotation", servo_spin);
        telemetry.addData("Arm Power", gamepad2.right_stick_y);
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
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

    BL_power *= REAR_RATIO;
    BR_power *= REAR_RATIO;

    FL.setPower(FL_power);
    BL.setPower(BL_power);
    FR.setPower(FR_power);
    BR.setPower(BR_power);
    
    
  }
}
  
