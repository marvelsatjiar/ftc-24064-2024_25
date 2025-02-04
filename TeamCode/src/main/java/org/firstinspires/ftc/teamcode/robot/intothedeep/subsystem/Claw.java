package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MAX;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MIN;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class Claw {
    private final double
            CLAMP_ANGLE = 0,
            DEPOSIT_ANGLE = 30;

    public enum ClawAngles {
        CLAMPED,
        WALL_PICKUP,
        DEPOSIT;

        public double getAngle() {
            switch (this) {
                case CLAMPED:               return 0;
                case WALL_PICKUP:         return 60;
                case DEPOSIT: default:    return 30;
            }
        }
    }

    private final ServoEx claw;

    private ClawAngles targetAngle = ClawAngles.DEPOSIT;

    public boolean isLocked = false;

    public Claw(HardwareMap hardwareMap) {
        claw = new SimpleServo(hardwareMap, "claw", SERVO_25_KG_MIN, SERVO_25_KG_MAX);
    }

    public boolean setAngle(ClawAngles angle, boolean isOverride) {
        if (isLocked && !isOverride) return false;

        targetAngle = angle;
        return true;
    }

    public boolean setAngle(ClawAngles angle) {
        return setAngle(angle, false);
    }

    public ClawAngles getClawAngle() {
        return targetAngle;
    }

    public void run() {
        claw.turnToAngle(targetAngle.getAngle());
    }
}

