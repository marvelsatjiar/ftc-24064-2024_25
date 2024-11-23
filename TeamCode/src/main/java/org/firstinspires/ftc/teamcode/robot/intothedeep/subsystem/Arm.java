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
            NEUTRAL_ARM_ANGLE = 76,
            TRANSFERRED_WRIST_ANGLE = 270,
            COLLECTING_ARM_ANGLE = 130,
            COLLECTING_WRIST_ANGLE = 105,
            BASKET_ARM_ANGLE = 26,
            BASKET_WRIST_ANGLE = 270,

            CHAMBER_FRONT_SETUP_ARM_ANGLE = 81,
            CHAMBER_FRONT_SCORE_ARM_ANGLE = 61,

            CHAMBER_BACK_SETUP_ARM_ANGLE = 26,
            CHAMBER_BACK_SCORE_ARM_ANGLE = 26,

            CHAMBER_FRONT_WRIST_ANGLE = 172.5,
            CHAMBER_BACK_WRIST_ANGLE = 210,

            WALL_PICKUP_ARM_ANGLE = 206,
            WALL_PICKUP_WRIST_ANGLE = 90;


    public enum WristAngle {
        COLLECTING,
        TRANSFERRED,
        BASKET,
        CHAMBER_FRONT,
        CHAMBER_BACK,
        WALL_PICKUP;

        public double getAngle() {
            switch (this) {
                case BASKET:                return BASKET_WRIST_ANGLE;
                case CHAMBER_BACK:          return CHAMBER_BACK_WRIST_ANGLE;
                case CHAMBER_FRONT:         return CHAMBER_FRONT_WRIST_ANGLE;
                case WALL_PICKUP:           return WALL_PICKUP_WRIST_ANGLE;
                case TRANSFERRED:           return TRANSFERRED_WRIST_ANGLE;
                case COLLECTING: default:   return COLLECTING_WRIST_ANGLE;
            }
        }
    }

    public enum ArmAngle {
        NEUTRAL,
        COLLECTING,
        BASKET,
        CHAMBER_FRONT_SETUP,
        CHAMBER_FRONT_SCORE,
        CHAMBER_BACK_SETUP,
        CHAMBER_BACK_SCORE,
        WALL_PICKUP;


        public double getAngle() {
            switch (this) {
                case BASKET:                return BASKET_ARM_ANGLE;
                case CHAMBER_FRONT_SETUP:   return CHAMBER_FRONT_SETUP_ARM_ANGLE;
                case CHAMBER_FRONT_SCORE:   return CHAMBER_FRONT_SCORE_ARM_ANGLE;
                case CHAMBER_BACK_SETUP:    return CHAMBER_BACK_SETUP_ARM_ANGLE;
                case CHAMBER_BACK_SCORE:    return CHAMBER_BACK_SCORE_ARM_ANGLE;
                case COLLECTING:            return COLLECTING_ARM_ANGLE;
                case WALL_PICKUP:           return WALL_PICKUP_ARM_ANGLE;
                case NEUTRAL: default:      return NEUTRAL_ARM_ANGLE;
            }
        }


    }
    private WristAngle targetWristAngle = WristAngle.COLLECTING;
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
        boolean isArmDown = getArmAngle() == ArmAngle.WALL_PICKUP;
        if (liftBelowSafety && isArmDown) targetArmAngle = ArmAngle.COLLECTING;

        for (ServoEx servos : armServos) {
            servos.turnToAngle(getArmAngle().getAngle());
        }

        wrist.turnToAngle(getWristAngle().getAngle());
    }

    public void printTelemetry() {
        mTelemetry.addData("ARM STATE:", targetArmAngle.name());
        mTelemetry.addData("WRIST STATE:", targetWristAngle.name());
    }
}
