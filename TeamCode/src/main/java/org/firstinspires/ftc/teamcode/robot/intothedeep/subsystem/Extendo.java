package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_AXON_MAX;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_AXON_MIN;
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
        LINKAGE_MAX_ANGLE = 130,
        STICK_MULT = 0.6;

    private double targetAngle = LINKAGE_MIN_ANGLE;

    public enum State {
        RETRACTED,
        EXTENDED;

        public double getExtendoAngle() {
            switch (this) {
                case RETRACTED:
                    default:
                        return LINKAGE_MIN_ANGLE;
                case EXTENDED:
                    return LINKAGE_MAX_ANGLE;
            }
        }
    }

    private State state = State.RETRACTED;

    public boolean isLocked = false;

    /**
     * Instantiates a new 'extendo' class that holds the extendo mechanisms (methods and functions)
     * @param hardwareMap A constant map that holds all the parts for config in code
     */
    public Extendo(HardwareMap hardwareMap) {
        SimpleServo masterLinkage = new SimpleServo(hardwareMap, "extendoLinkageMain", SERVO_AXON_MIN, SERVO_AXON_MAX);
        SimpleServo followerLinkage = new SimpleServo(hardwareMap, "extendoLinkageFollower", SERVO_AXON_MIN, SERVO_AXON_MAX);

        masterLinkage.setInverted(true);

        linkageServos = new SimpleServo[]{masterLinkage, followerLinkage};
    }

    // Returns the state of the extendo
    public State getState() {
        return state;
    }

    public boolean setTargetAngle(State angle, boolean isOverride) {
        if (isLocked && !isOverride) return false;
        targetAngle = min(LINKAGE_MAX_ANGLE, max(LINKAGE_MIN_ANGLE, angle.getExtendoAngle()));
        state = targetAngle == LINKAGE_MIN_ANGLE ?
                State.RETRACTED :
                State.EXTENDED;

        return true;
    }

    public boolean setTargetAngle(State state) {
        return setTargetAngle(state, false);
    }

    // Runs each servo inside of the group to a certain angle base on what was given
    public boolean run(boolean isV4BUnsafe) {
        if (isV4BUnsafe) return false;
        for (SimpleServo servos : linkageServos) {
            servos.turnToAngle(targetAngle);
        }

        return true;
    }

    // Prints data on the driver hub for debugging and other uses
    public void printTelemetry() {
        mTelemetry.addData("Extendo is", state);
        mTelemetry.addData("Servo angle is", targetAngle);
    }
}
