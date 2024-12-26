package org.firstinspires.ftc.teamcode.sensor.vision;

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

    private final ArrayList<SampleColor> readings = new ArrayList<>();
    private SampleColor reading;
    private SampleColor rawReading;

    public static int QUEUE_LENGTH = 5;

    public ColorRangefinderEx(HardwareMap hardwareMap) {
        pin0 = hardwareMap.digitalChannel.get("digital0");
        pin1 = hardwareMap.digitalChannel.get("digital1");

        for (int i = 0; i < QUEUE_LENGTH; i++) {
            readings.add(SampleColor.NOTHING);
        }

        reading = SampleColor.NOTHING;
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

    public SampleColor getReading() {
        return reading;
    }
    public SampleColor getRawReading() {
        return rawReading;
    }
    public SampleColor run() {
        readings.add(convertToEnum());
        rawReading = readings.get(readings.size() - 1);

        if (readings.size() > QUEUE_LENGTH) {
            readings.remove(0);
        }

        reading = interpretReadings();
        return reading;
    }

    private SampleColor interpretReadings() {
        int redCounter = 0;
        int blueCounter = 0;
        int yellowCounter = 0;
        int nothingCounter = 0;
        for (int i = 0; i < QUEUE_LENGTH; i++) {
            switch (readings.get(i)) {
                case RED: redCounter++;
                case BLUE: blueCounter++;
                case YELLOW: yellowCounter++;
                case NOTHING: nothingCounter++;
            }
        }

        if (redCounter > blueCounter && redCounter > yellowCounter && redCounter > nothingCounter) {
            return SampleColor.RED;
        } else if (blueCounter > redCounter && blueCounter > yellowCounter && blueCounter > nothingCounter)
            return SampleColor.BLUE;
        else if (yellowCounter > redCounter && yellowCounter > blueCounter && yellowCounter > nothingCounter)
            return SampleColor.YELLOW;
        else if (nothingCounter > redCounter && nothingCounter > blueCounter && nothingCounter > yellowCounter)
            return SampleColor.NOTHING;
        else return SampleColor.NOTHING;
    }
}
