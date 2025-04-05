package org.firstinspires.ftc.teamcode.sensor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

@Config
public class ColorRangefinderEx {
    public final DigitalChannel
            pin0,
            pin1;

    public enum SampleColor {
        YELLOW,
        BLUE,
        RED,
        NOTHING;
    }

    public enum Modes {
        ANALOG,
        DIGITAL
    }

    private SampleColor rawReading = SampleColor.NOTHING;

    public ColorRangefinderEx(HardwareMap hardwareMap) {
        pin0 = hardwareMap.digitalChannel.get("digital0");
        pin1 = hardwareMap.digitalChannel.get("digital1");

    }

    public SampleColor convertToEnum() {
        if (pin0.getState()) {
            if (pin1.getState()) return SampleColor.YELLOW;
            if (!pin1.getState()) return SampleColor.BLUE;
        }

        if (pin1.getState()) {
            if (!pin0.getState()) return SampleColor.RED;
        }
        return SampleColor.NOTHING;
    }

    public SampleColor getRawReading() {
        return rawReading;
    }
    public SampleColor run() {
        rawReading = convertToEnum();
        return rawReading;
    }
}
