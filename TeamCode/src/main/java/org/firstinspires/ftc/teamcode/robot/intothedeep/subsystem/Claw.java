package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.SimpleServoPivot;

public final class Claw {
    private final double
            CLAMP_ANGLE = 0,
            DEPOSIT_ANGLE = 30;

    private final SimpleServoPivot claw;

    public boolean isLocked = false;

    public Claw(HardwareMap hardwareMap) {
        claw = new SimpleServoPivot(DEPOSIT_ANGLE, CLAMP_ANGLE, SimpleServoPivot.getGoBildaServo(hardwareMap, "claw"));
    }

    public boolean setClamped(boolean isActivated, boolean isOverride) {
        if (isLocked && !isOverride) return false;

        claw.setActivated(isActivated);
        return true;
    }

    public boolean setClamped(boolean isActivated) {
        return setClamped(isActivated, false);
    }

    public boolean getClamped() {
        return claw.isActivated();
    }

    public void run() {
        claw.updateAngles(DEPOSIT_ANGLE, CLAMP_ANGLE);
        claw.run();
    }
}

