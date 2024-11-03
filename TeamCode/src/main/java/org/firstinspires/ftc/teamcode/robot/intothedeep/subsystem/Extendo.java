package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class Extendo {
    // Constants and classes made to make the subssystem work properly
    SimpleServo[] linkageServos;

    public static double
        LINKAGE_MIN_ANGLE = 100,
        LINKAGE_MAX_ANGLE = 110,
        STICK_MULT = 0.2;

    private double targetAngle = LINKAGE_MIN_ANGLE;

    public enum ExtendoState {
        RETRACTED,
        RUNNING
    }

    private ExtendoState state = ExtendoState.RETRACTED;

    public boolean isLocked = false;

    /**
     * Instantiates a new 'extendo' class that holds the extendo mechanisms (methods and functions)
     * @param hardwareMap A constant map that holds all the parts for config in code
     */
    public Extendo(HardwareMap hardwareMap) {
        SimpleServo masterLinkage = new SimpleServo(hardwareMap, "extendoLinkageMain", LINKAGE_MIN_ANGLE, LINKAGE_MAX_ANGLE);
        SimpleServo followerLinkage = new SimpleServo(hardwareMap, "extendoLinkageFollower", LINKAGE_MIN_ANGLE, LINKAGE_MAX_ANGLE);

        masterLinkage.setInverted(true);

        linkageServos = new SimpleServo[]{masterLinkage, followerLinkage};
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Allows for the driver to set the extendo's height with the joystick
     * It uses the min and max methods w/ some set minimums and maximums for how far the servo can go
     * It will also set the state of the extendo
     * @param stick The value the joystick is giving to the code which allows us to control it with ease
     */
    public void setWithStick(double stick) {
        this.setTargetAngle(getTargetAngle() + stick * STICK_MULT);
    }

    // Returns the state of the extendo
    public ExtendoState getState() {
        return state;
    }

    public boolean setTargetAngle(double angle, boolean isOverride) {
        if (isLocked && !isOverride) return false;
        targetAngle = min(LINKAGE_MAX_ANGLE, max(LINKAGE_MIN_ANGLE, angle));
        state = getTargetAngle() == LINKAGE_MIN_ANGLE ?
                ExtendoState.RETRACTED :
                ExtendoState.RUNNING;

        return true;
    }

    public boolean setTargetAngle(double angle) {
        return setTargetAngle(angle, false);
    }

    // Runs each servo inside of the group to a certain angle base on what was given
    public boolean run(boolean isV4BUnsafe) {
        if (isV4BUnsafe) return false;
        for (SimpleServo servos : linkageServos) {
            servos.turnToAngle(getTargetAngle());
        }

        return true;
    }

    // Prints data on the driver hub for debugging and other uses
    public void printTelemetry() {
        mTelemetry.addData("Extendo is: ", state);
        mTelemetry.addData("Servo angle is: ", getTargetAngle());
    }
}
