package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "ColorTest", group = "")
public class ColorTest extends LinearOpMode {
    private ColorSensor col_sensor;
    
    @Override
    public void runOpMode() {
        col_sensor = hardwareMap.colorSensor.get("color_sensor");
        
        int red, green, blue;
        
        //Get within one inch of sensor for maximum effectiveness
        
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                red = col_sensor.red();
                green = col_sensor.green();
                blue = col_sensor.blue();
                
                telemetry.addData("red", red);
                telemetry.addData("green", green);
                telemetry.addData("blue", blue);
                telemetry.update();
            }
        }
    }
}