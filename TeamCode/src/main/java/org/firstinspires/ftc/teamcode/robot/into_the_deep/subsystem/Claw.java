package org.firstinspires.ftc.teamcode.robot.into_the_deep.subsystem;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.SimpleServoPivot;

public class Claw {

    private double
        CLAMPANGLE = 0,
        DEPOSITANGLE = 60;


    SimpleServoPivot masterClaw;
    SimpleServoPivot followerClaw;

    SimpleServoPivot[] servoGroup;

    public Claw(HardwareMap hardwareMap) {
        masterClaw = new SimpleServoPivot(DEPOSITANGLE, CLAMPANGLE, SimpleServoPivot.getGoBildaServo(hardwareMap, "masterClaw"));
        followerClaw = new SimpleServoPivot(DEPOSITANGLE, CLAMPANGLE, SimpleServoPivot.getGoBildaServo(hardwareMap, "followerClaw"));

        servoGroup = new SimpleServoPivot[] {masterClaw, followerClaw};
    }

    public void setClaw(boolean isPushed) {
        for (SimpleServoPivot servos: servoGroup) {
        servos.setActivated(isPushed);
    }}

    public void toggleClaw(){
        for (SimpleServoPivot servos: servoGroup) {servos.toggle();}
    }


    public void run(){
        for (SimpleServoPivot servos: servoGroup) {
            servos.updateAngles(DEPOSITANGLE, CLAMPANGLE);
            servos.run();
        }
    }
}

