package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.opmode.MainTeleOp.mTelemetry;
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
        LINKAGE_MIN_ANGLE = 2,
        LINKAGE_MAX_ANGLE = 90,
        STICK_MULT = 2;

    private double linkageTargetAngle = 0;

    public enum ExtendoState {
        RETRACTED,
        RUNNING
    }

    private ExtendoState state;

    /**
     * Instantiates a new 'extendo' class that holds the extendo mechanisms (methods and functions)
     * @param hw A constant map that holds all the parts for config in code
     */
    public Extendo(HardwareMap hw) {
        SimpleServo masterLinkage = new SimpleServo(hw, "extendoLinkageMain", LINKAGE_MIN_ANGLE, LINKAGE_MAX_ANGLE);
        SimpleServo followerLinkage = new SimpleServo(hw, "extendoLinkageFollower", LINKAGE_MIN_ANGLE, LINKAGE_MAX_ANGLE);

        linkageServos = new SimpleServo[]{masterLinkage, followerLinkage};
    }

    /**
     * Allows for the driver to set the extendo's height with the joystick
     * It uses the min and max methods w/ some set minimums and maximums for how far the servo can go
     * It will also set the state of the extendo
     * @param stick The value the joystick is giving to the code which allows us to control it with ease
     */
    public void setWithStick(double stick) {
        linkageTargetAngle = min(LINKAGE_MIN_ANGLE, max(LINKAGE_MAX_ANGLE, linkageTargetAngle + stick * STICK_MULT));
        state = linkageTargetAngle == LINKAGE_MIN_ANGLE ?
                ExtendoState.RETRACTED :
                ExtendoState.RUNNING;
    }

    // Returns the state of the extendo
    public ExtendoState getState() {
        return state;
    }

    // Runs each servo inside of the group to a certain angle base on what was given
    public void run() {
        for (SimpleServo servos : linkageServos) {servos.turnToAngle(linkageTargetAngle);}
    }

    // Prints data on the driver hub for debugging and other uses
    public void printTelemetry() {
        mTelemetry.addData("Extendo is: ", state);
        mTelemetry.addData("Servo angle is: ", linkageTargetAngle);
    }
}
