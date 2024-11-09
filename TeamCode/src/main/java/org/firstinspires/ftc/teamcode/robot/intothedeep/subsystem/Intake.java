package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MAX;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MIN;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class Intake {
    private final CRServo[] intakeGroup;
    private final ServoEx[] intakeLinkGroup;

    public static int
            V4B_DOWN_ANGLE = 130,
            V4B_CLEARING_ANGLE = 115,
            V4B_UP_ANGLE = 110,
            V4B_UNSAFE_THRESHOLD_ANGLE = 111;

    private V4BAngle targetAngle = V4BAngle.UP;

    public enum V4BAngle {
        DOWN,
        CLEARING,
        UP,
        UNSAFE;

        private int getAngle() {
            switch (this) {
                case DOWN: return  V4B_DOWN_ANGLE;
                case CLEARING: return V4B_CLEARING_ANGLE;
                case UNSAFE: return V4B_UNSAFE_THRESHOLD_ANGLE;
                default: return V4B_UP_ANGLE;
            }
        }

        public boolean isV4BUnsafe() {
            return getAngle() >= V4B_UNSAFE_THRESHOLD_ANGLE;
        }
    }

    public boolean isV4BLocked = false;

    public boolean isRollerLocked = false;

    public Intake(HardwareMap hardwareMap) {
        CRServo intakeFollower = new CRServo(hardwareMap, "intakeFollower");
        CRServo intakeMaster = new CRServo(hardwareMap, "intakeMaster");
        ServoEx intakeGearFollower = new SimpleServo(hardwareMap, "intakeLinkFollower", SERVO_25_KG_MIN, SERVO_25_KG_MAX);
        ServoEx intakeGearMaster = new SimpleServo(hardwareMap, "intakeLinkMaster", SERVO_25_KG_MIN, SERVO_25_KG_MAX);

        intakeFollower.setInverted(true);
        intakeGearMaster.setInverted(true);

        intakeGroup = new CRServo[] {intakeFollower, intakeMaster};
        intakeLinkGroup = new ServoEx[] {intakeGearFollower, intakeGearMaster};
    }

    public boolean setTargetV4BAngle(V4BAngle angle, boolean isOverride) {
        if (isV4BLocked && !isOverride) return false;
        targetAngle = angle;

        return true;
    }

    public boolean setTargetV4BAngle(V4BAngle angle) {
        return setTargetV4BAngle(angle, false);
    }

    public V4BAngle getTargetV4BAngle() {
        return targetAngle;
    }

    public boolean setRollerPower(double power, boolean isOverride) {
        if (isRollerLocked && !isOverride) return false;

        for (CRServo servos : intakeGroup)
            servos.set(power);

        return true;
    }

    public boolean setRollerPower(double power) {
        return setRollerPower(power, false);
    }

    public void run() {
        for (ServoEx servos : intakeLinkGroup)
            servos.turnToAngle(targetAngle.getAngle());
    }
}
