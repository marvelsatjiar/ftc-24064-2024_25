package org.firstinspires.ftc.teamcode.robot.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.sensor.ColorRangefinder;

@TeleOp
@Config
public class ConfigureColorRangefinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorRangefinder crf =
                new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "Color"));

        /*
        Using this example configuration, you can detect all three sample colors based on which pin is reading true:
        both      --> yellow
        only pin0 --> red
        only pin1 --> blue
        neither   --> no object
         */
        crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 180 / 360.0 * 255, 250 / 360.0 * 255); // blue
        crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 55 / 360.0 * 255, 75 / 360.0 * 255); // yellow
        crf.setPin0DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 35); // 20mm or closer requirement

        crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 140 / 360.0 * 255, 210 / 360.0 * 255); // inverted red
        crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 235 / 360.0 * 255, 255 / 360.0 * 255); // inverted yellow
        crf.setPin1DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 35); // 20mm or closer requirement
        crf.setPin1InvertHue(); // invert hue values

        waitForStart();

        stop();
    }
}
