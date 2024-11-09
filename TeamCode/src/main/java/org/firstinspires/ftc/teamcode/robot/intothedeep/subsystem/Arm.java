package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MAX;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MIN;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_45_KG_MAX;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_45_KG_MIN;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class Arm {
    private final ServoEx wrist;

    private final ServoEx[] armServos;

    public static double
            NEUTRAL_ARM_ANGLE = 50,
            NEUTRAL_WRIST_ANGLE = 270,
            COLLECTING_ARM_ANGLE = 104,
            COLLECTING_WRIST_ANGLE = 110,
            BASKET_ARM_ANGLE = 0,
            BASKET_WRIST_ANGLE = 270,
            CHAMBER_ARM_ANGLE = 70,
            CHAMBER_WRIST_ANGLE = 180;

    public enum WristAngle {
        NEUTRAL,
        COLLECTING,
        BASKET,
        CHAMBER;

        public double getAngle() {
            switch (this) {
                case BASKET:            return BASKET_WRIST_ANGLE;
                case CHAMBER:           return CHAMBER_WRIST_ANGLE;
                case COLLECTING:        return COLLECTING_WRIST_ANGLE;
                case NEUTRAL: default:  return NEUTRAL_WRIST_ANGLE;
            }
        }
    }

    public enum ArmAngle {
        NEUTRAL,
        COLLECTING,
        BASKET,
        CHAMBER;

        public double getAngle() {
            switch (this) {
                case BASKET:            return BASKET_ARM_ANGLE;
                case CHAMBER:           return CHAMBER_ARM_ANGLE;
                case COLLECTING:        return COLLECTING_ARM_ANGLE;
                case NEUTRAL: default:  return NEUTRAL_ARM_ANGLE;
            }
        }


    }
    private WristAngle targetWristAngle = WristAngle.NEUTRAL;
    private ArmAngle targetArmAngle = ArmAngle.NEUTRAL;

    public boolean isLocked = false;

    public Arm(HardwareMap hardwareMap) {
        wrist = new SimpleServo(hardwareMap, "wrist", SERVO_25_KG_MIN, SERVO_25_KG_MAX);
        armServos = new ServoEx[] {
                new SimpleServo(hardwareMap, "arm master", SERVO_45_KG_MIN, SERVO_45_KG_MAX),
                new SimpleServo(hardwareMap, "arm follower", SERVO_45_KG_MIN, SERVO_45_KG_MAX)
        };

        armServos[1].setInverted(true);
    }

    public boolean setArmAngle(ArmAngle angle, boolean isOverride) {
        if (isLocked && !isOverride) return false;
        targetArmAngle = angle;

        return true;
    }

    public boolean setArmAngle(ArmAngle angle) {
        return setArmAngle(angle, false);
    }

    public boolean setWristAngle(WristAngle angle, boolean isOverride) {
        if (isLocked && !isOverride) return false;
        targetWristAngle = angle;

        return true;
    }

    public boolean setWristAngle(WristAngle angle) {
        return setWristAngle(angle, false);
    }

    public ArmAngle getArmAngle() {
        return targetArmAngle;
    }

    public WristAngle getWristAngle() {
        return targetWristAngle;
    }

     public void run(boolean liftBelowSafety) {
        boolean isArmDown = getArmAngle() == ArmAngle.CHAMBER;
        if (liftBelowSafety && isArmDown) targetArmAngle = ArmAngle.COLLECTING;

        for (ServoEx servos : armServos) {
            servos.turnToAngle(getArmAngle().getAngle());
        }

        wrist.turnToAngle(getWristAngle().getAngle());
    }

    public void printTelemetry() {
        mTelemetry.addData("ARM STATE:", targetArmAngle.getAngle());
        mTelemetry.addData("WRIST STATE:", targetWristAngle.getAngle());
    }
}
