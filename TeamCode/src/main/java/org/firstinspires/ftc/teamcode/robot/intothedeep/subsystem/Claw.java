package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.SimpleServoPivot;

public final class Claw {
    private final double
            CLAMP_ANGLE = 0,
            DEPOSIT_ANGLE = 30;
    private SimpleServoPivot claw;

    public Claw(HardwareMap hardwareMap) {
        claw = new SimpleServoPivot(DEPOSIT_ANGLE, CLAMP_ANGLE, SimpleServoPivot.getGoBildaServo(hardwareMap, "claw"));
    }

    public void setClaw(boolean isPushed) {
        claw.setActivated(isPushed);
    }

    public void toggleClaw(){
        claw.toggle();
    }


    public void run(){
        claw.updateAngles(DEPOSIT_ANGLE, CLAMP_ANGLE);
        claw.run();
    }
}

