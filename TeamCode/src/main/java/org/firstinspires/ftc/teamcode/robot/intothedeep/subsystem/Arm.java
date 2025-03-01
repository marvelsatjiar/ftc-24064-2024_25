package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MAX;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MIN;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_45_KG_MAX;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_45_KG_MIN;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_AXON_MAX;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_AXON_MIN;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class Arm {
    private final ServoEx wrist, armstendo;

    private final ServoEx[] armServos;

    public static double
            NEUTRAL_ARM_ANGLE = 195,
            TRANSFERRED_WRIST_ANGLE = 220,
            COLLECTING_ARM_ANGLE = 227,
            COLLECTING_WRIST_ANGLE = 45,
            BASKET_ARM_ANGLE = 105,
            BASKET_WRIST_ANGLE = 220,

            FRONT_WALL_PICKUP_ARM_ANGLE = 220,
            FRONT_WALL_PICKUP_WRIST_ANGLE = 135,

            FRONT_WALL_SPECIMEN_SETUP_ARM_ANGLE = 120,
            FRONT_WALL_SPECIMEN_SETUP_WRIST_ANGLE = 200,
            OVERHANG_SPECIMEN_SETUP_ARM_ANGLE = 50,
            OVERHANG_SPECIMEN_SETUP_WRIST_ANGLE = 255,

            BEFORE_OVERHANG_SPECIMEN_ARM_ANGLE = 200,

            FRONT_WALL_SPECIMEN_SCORE_ARM_ANGLE = 160,
            FRONT_WALL_SPECIMEN_SCORE_WRIST_ANGLE = 150,

            BACK_WALL_PICKUP_ARM_ANGLE = 170,
            BACK_WALL_PICKUP_WRIST_ANGLE = 270,

            CHAMBER_FRONT_SETUP_ARM_ANGLE = 175,

            BACK_WALL_SPECIMEN_SETUP_ARM_ANGLE = 270,
            BACK_WALL_SPECIMEN_SETUP_WRIST_ANGLE = 90,

            WALL_PICKUP_ARMSTENDO_ANGLE = 50,
            EXTENDED_ARMSTENDO_ANGLE = 90,
            TRANSFER_ARMSTENDO_ANGLE = 30,
            RETRACTED_ARMSTENDO_ANGLE = 5,

            UNSAFE_ARM_THRESHOLD = 240;

    public enum WristAngle {
        COLLECTING,
        FRONT_WALL_PICKUP,
        FRONT_WALL_SPECIMEN_SETUP,
        FRONT_WALL_SPECIMEN_SCORE,
        BACK_WALL_PICKUP,
        SETUP_SPECIMEN_WITH_ARMSTENDO,
        TRANSFERRED,
        BASKET,
        OVERHANG_SPECIMEN_SETUP;

        public double getAngle() {
            switch (this) {
                case BASKET:                        return BASKET_WRIST_ANGLE;
                case FRONT_WALL_PICKUP:             return FRONT_WALL_PICKUP_WRIST_ANGLE;
                case BACK_WALL_PICKUP:              return BACK_WALL_PICKUP_WRIST_ANGLE;
                case SETUP_SPECIMEN_WITH_ARMSTENDO:      return BACK_WALL_SPECIMEN_SETUP_WRIST_ANGLE;
                case FRONT_WALL_SPECIMEN_SETUP:     return FRONT_WALL_SPECIMEN_SETUP_WRIST_ANGLE;
                case FRONT_WALL_SPECIMEN_SCORE:     return FRONT_WALL_SPECIMEN_SCORE_WRIST_ANGLE;
                case OVERHANG_SPECIMEN_SETUP:       return OVERHANG_SPECIMEN_SETUP_WRIST_ANGLE;
                case TRANSFERRED:                   return TRANSFERRED_WRIST_ANGLE;
                case COLLECTING: default:           return COLLECTING_WRIST_ANGLE;
            }
        }
    }

    public enum ArmAngle {
        NEUTRAL,
        FRONT_WALL_PICKUP,
        CHAMBER_FRONT_SETUP,
        FRONT_WALL_SPECIMEN_SETUP,
        FRONT_WALL_SPECIMEN_SCORE,
        BACK_WALL_PICKUP,
        SETUP_SPECIMEN_WITH_ARMSTENDO,
        COLLECTING,
        BASKET,
        OVERHANG_SPECIMEN_SETUP,
        BEFORE_OVERHANG_SPECIMEN;

        public double getAngle() {
            switch (this) {
                case BASKET:                    return BASKET_ARM_ANGLE;
                case FRONT_WALL_PICKUP:         return FRONT_WALL_PICKUP_ARM_ANGLE;
                case BACK_WALL_PICKUP:          return BACK_WALL_PICKUP_ARM_ANGLE;
                case SETUP_SPECIMEN_WITH_ARMSTENDO:  return BACK_WALL_SPECIMEN_SETUP_ARM_ANGLE;
                case FRONT_WALL_SPECIMEN_SETUP: return FRONT_WALL_SPECIMEN_SETUP_ARM_ANGLE;
                case FRONT_WALL_SPECIMEN_SCORE: return FRONT_WALL_SPECIMEN_SCORE_ARM_ANGLE;
                case CHAMBER_FRONT_SETUP:       return CHAMBER_FRONT_SETUP_ARM_ANGLE;
                case BEFORE_OVERHANG_SPECIMEN:  return BEFORE_OVERHANG_SPECIMEN_ARM_ANGLE;
                case OVERHANG_SPECIMEN_SETUP:   return OVERHANG_SPECIMEN_SETUP_ARM_ANGLE;
                case COLLECTING:                return COLLECTING_ARM_ANGLE;
                case NEUTRAL: default:          return NEUTRAL_ARM_ANGLE;
            }
        }


    }

    public enum Extension {
        RETRACTED,
        EXTENDED,
        WALl_PICKUP,
        TRANSFER;

        public double getAngle() {
            switch (this) {
                case EXTENDED:                  return EXTENDED_ARMSTENDO_ANGLE;
                case WALl_PICKUP:               return WALL_PICKUP_ARMSTENDO_ANGLE;
                case TRANSFER:                  return TRANSFER_ARMSTENDO_ANGLE;
                case RETRACTED: default:        return RETRACTED_ARMSTENDO_ANGLE;
            }
        }
    }


    private WristAngle targetWristAngle = WristAngle.COLLECTING;
    private ArmAngle targetArmAngle = ArmAngle.NEUTRAL;
    private Extension targetArmstendoExtension = Extension.RETRACTED;
    public boolean isLocked = false;

    public Arm(HardwareMap hardwareMap) {
        wrist = new SimpleServo(hardwareMap, "wrist", SERVO_25_KG_MIN, SERVO_25_KG_MAX);
        armstendo = new SimpleServo(hardwareMap, "armstendo", SERVO_45_KG_MIN, SERVO_45_KG_MAX);
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

    public boolean setArmstendoAngle(Extension extension, boolean isOverride) {
        if (isLocked && !isOverride) return false;
        targetArmstendoExtension = extension;

        return true;
    }

    public boolean setArmstendoAngle(Extension extension) {
        return setArmstendoAngle(extension, false);
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

    public Extension getArmstendoAngle() {return targetArmstendoExtension;}

    public WristAngle getWristAngle() {
        return targetWristAngle;
    }

     public void run() {
        wrist.turnToAngle(getWristAngle().getAngle());

        armstendo.turnToAngle(getArmstendoAngle().getAngle());

         for (ServoEx servos : armServos) {
            servos.turnToAngle(getArmAngle().getAngle());
        }
    }

    public void printTelemetry() {
        mTelemetry.addData("ARM STATE:", targetArmAngle.name());
        mTelemetry.addData("WRIST STATE:", targetWristAngle.name());
    }
}
