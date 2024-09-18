package org.firstinspires.ftc.teamcode.robot.into_the_deep.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.SimpleServoPivot;

public class Claw {
    private final double
        CLAMP_ANGLE = 0,
        DEPOSIT_ANGLE = 60;
    SimpleServoPivot
            masterClaw,
            followerClaw;
    SimpleServoPivot[] servoGroup;

    public Claw(HardwareMap hardwareMap) {
        masterClaw = new SimpleServoPivot(DEPOSIT_ANGLE, CLAMP_ANGLE, SimpleServoPivot.getGoBildaServo(hardwareMap, "masterClaw"));
        followerClaw = new SimpleServoPivot(DEPOSIT_ANGLE, CLAMP_ANGLE, SimpleServoPivot.getGoBildaServo(hardwareMap, "followerClaw"));

        servoGroup = new SimpleServoPivot[] {masterClaw, followerClaw};
    }

    public void setClaw(boolean isPushed) {
        for (SimpleServoPivot servos: servoGroup) {
        servos.setActivated(isPushed);
        }
    }

    public void toggleClaw(){
        for (SimpleServoPivot servos: servoGroup)
            servos.toggle();
    }


    public void run(){
        for (SimpleServoPivot servos: servoGroup) {
            servos.updateAngles(DEPOSIT_ANGLE, CLAMP_ANGLE);
            servos.run();
        }
    }
}

