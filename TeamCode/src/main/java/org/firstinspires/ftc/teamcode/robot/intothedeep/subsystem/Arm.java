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
            NEUTRAL_ARM_ANGLE = 80,
            NEUTRAL_WRIST_ANGLE = 110,
            COLLECTING_ARM_ANGLE = 104,
            COLLECTING_WRIST_ANGLE = 110,
            HIGH_BASKET_ARM_ANGLE = 60,
            HIGH_BASKET_WRIST_ANGLE = 50,
            LOW_BASKET_ARM_ANGLE = 60,
            LOW_BASKET_WRIST_ANGLE = 50,
            HIGH_CHAMBER_UPWARDS_ARM_ANGLE = 160,
            HIGH_CHAMBER_UPWARDS_WRIST_ANGLE = 200,
            HIGH_CHAMBER_DOWNWARDS_ARM_ANGLE = 40,
            HIGH_CHAMBER_DOWNWARDS_WRIST_ANGLE = 200;

    public enum Position {
        NEUTRAL,
        COLLECTING,
        HIGH_BASKET,
        LOW_BASKET,
        HIGH_CHAMBER_UPWARDS,
        HIGH_CHAMBER_DOWNWARDS;

        public double getArmAngle() {
            switch (this) {
                case HIGH_BASKET: return HIGH_BASKET_ARM_ANGLE;
                case LOW_BASKET: return LOW_BASKET_ARM_ANGLE;
                case HIGH_CHAMBER_UPWARDS: return HIGH_CHAMBER_UPWARDS_ARM_ANGLE;
                case HIGH_CHAMBER_DOWNWARDS: return HIGH_CHAMBER_DOWNWARDS_ARM_ANGLE;
                case COLLECTING: return COLLECTING_ARM_ANGLE;
                case NEUTRAL:
                default:
                    return NEUTRAL_ARM_ANGLE;
            }
        }

        public double getWristAngle() {
            switch (this) {
                case HIGH_BASKET: return HIGH_BASKET_WRIST_ANGLE;
                case LOW_BASKET: return LOW_BASKET_WRIST_ANGLE;
                case HIGH_CHAMBER_UPWARDS: return HIGH_CHAMBER_UPWARDS_WRIST_ANGLE;
                case HIGH_CHAMBER_DOWNWARDS: return HIGH_CHAMBER_DOWNWARDS_WRIST_ANGLE;
                case COLLECTING: return COLLECTING_WRIST_ANGLE;
                case NEUTRAL:
                default:
                    return NEUTRAL_WRIST_ANGLE;
            }
        }
    }

    private Position targetPosition;

    public boolean isLocked = false;

    public Arm(HardwareMap hardwareMap) {
        wrist = new SimpleServo(hardwareMap, "wrist", SERVO_25_KG_MIN, SERVO_25_KG_MAX);
        armServos = new ServoEx[] {
                new SimpleServo(hardwareMap, "arm master", SERVO_45_KG_MIN, SERVO_45_KG_MAX),
                new SimpleServo(hardwareMap, "arm follower", SERVO_45_KG_MIN, SERVO_45_KG_MAX)
        };

        armServos[1].setInverted(true);
    }

    public boolean setTargetPosition(Position angle, boolean isOverride) {
        if (isLocked && !isOverride) return false;
        targetPosition = angle;

        return true;
    }

    public boolean setTargetPosition(Position angle) {
        return setTargetPosition(angle, false);
    }

    public Position getTargetPosition() {
        return targetPosition;
    }

     public void run(boolean liftBelowSafety) {
        boolean isArmDown = getTargetPosition() == Position.HIGH_CHAMBER_DOWNWARDS;
        if (liftBelowSafety && isArmDown) targetPosition = Position.COLLECTING;

        for (ServoEx servos : armServos) {
            servos.turnToAngle(getTargetPosition().getArmAngle());
        }

        wrist.turnToAngle(getTargetPosition().getWristAngle());
    }

    public void printTelemetry() {
        mTelemetry.addData("ARM STATE:", targetPosition.getArmAngle());
        mTelemetry.addData("WRIST STATE:", targetPosition.getWristAngle());
    }
}
