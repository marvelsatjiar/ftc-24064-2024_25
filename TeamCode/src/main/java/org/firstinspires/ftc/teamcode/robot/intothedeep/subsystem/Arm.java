package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class Arm {
    private final ServoEx wrist;

    private final ServoEx[] armServos;

    public static double
            COLLECTING_ARM_ANGLE = 100,
            COLLECTING_WRIST_ANGLE = 10,
            HIGH_BASKET_ARM_ANGLE = 60,
            HIGH_BASKET_WRIST_ANGLE = 50,
            LOW_BASKET_ARM_ANGLE = 60,
            LOW_BASKET_WRIST_ANGLE = 50,
            HIGH_CHAMBER_UPWARDS_ARM_ANGLE = 160,
            HIGH_CHAMBER_UPWARDS_WRIST_ANGLE = 200,
            HIGH_CHAMBER_DOWNWARDS_ARM_ANGLE = 40,
            HIGH_CHAMBER_DOWNWARDS_WRIST_ANGLE = 200;

    public enum Position {
        COLLECTING,
        HIGH_BASKET,
        LOW_BASKET,
        HIGH_CHAMBER_UPWARDS,
        HIGH_CHAMBER_DOWNWARDS;

        double getArmAngle() {
            switch (this) {
                case HIGH_BASKET: return HIGH_BASKET_ARM_ANGLE;
                case LOW_BASKET: return LOW_BASKET_ARM_ANGLE;
                case HIGH_CHAMBER_UPWARDS: return HIGH_CHAMBER_UPWARDS_ARM_ANGLE;
                case HIGH_CHAMBER_DOWNWARDS: return HIGH_CHAMBER_DOWNWARDS_ARM_ANGLE;
                case COLLECTING:
                default:
                    return COLLECTING_ARM_ANGLE;
            }
        }

        double getWristAngle() {
            switch (this) {
                case HIGH_BASKET: return HIGH_BASKET_WRIST_ANGLE;
                case LOW_BASKET: return LOW_BASKET_WRIST_ANGLE;
                case HIGH_CHAMBER_UPWARDS: return HIGH_CHAMBER_UPWARDS_WRIST_ANGLE;
                case HIGH_CHAMBER_DOWNWARDS: return HIGH_CHAMBER_DOWNWARDS_WRIST_ANGLE;
                case COLLECTING:
                default:
                    return COLLECTING_WRIST_ANGLE;
            }
        }
    }

    private Position targetPosition;

    public Arm(HardwareMap hardwareMap) {
        wrist = new SimpleServo(hardwareMap, "wrist", 0, 180);
        armServos = new ServoEx[]{
                new SimpleServo(hardwareMap, "arm master", 0, 240),
                new SimpleServo(hardwareMap, "arm follower", 0, 240)
        };

        armServos[1].setInverted(true);
    }

    public void setTarget(Position angle) {
        targetPosition = angle;
    }

     void run(boolean liftBelowSafety) {
         boolean isArmDown = targetPosition == Position.HIGH_CHAMBER_DOWNWARDS;
         if (liftBelowSafety && isArmDown) targetPosition = Position.COLLECTING;

        for (ServoEx servos : armServos) {
            servos.turnToAngle(targetPosition.getArmAngle());
        }
        wrist.turnToAngle(targetPosition.getWristAngle());
    }
}
