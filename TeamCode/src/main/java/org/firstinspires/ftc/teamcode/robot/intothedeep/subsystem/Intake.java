package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

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

    private V4BAngles targetAngle = V4BAngles.UP;

    public enum V4BAngles {
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
    public Intake(HardwareMap hardwareMap) {
        CRServo intakeFollower = new CRServo(hardwareMap, "intakeFollower");
        CRServo intakeMaster = new CRServo(hardwareMap, "intakeMaster");
        ServoEx intakeGearFollower = new SimpleServo(hardwareMap, "intakeLinkFollower", 0, 180);
        ServoEx intakeGearMaster = new SimpleServo(hardwareMap, "intakeLinkMaster", 0, 180);

        intakeFollower.setInverted(true);
        intakeGearFollower.setInverted(true);

        intakeGroup = new CRServo[] {intakeFollower, intakeMaster};
        intakeLinkGroup = new ServoEx[] {intakeGearFollower, intakeGearMaster};
    }

    public void setTarget(V4BAngles angle) {
        targetAngle = angle;
    }

    public Intake.V4BAngles getTargetAngle() {
        return targetAngle;
    }

    public void setServoPower(double power) {
        for (CRServo servos : intakeGroup)
            servos.set(power);
    }
    public void run() {
        for (ServoEx servos : intakeLinkGroup)
            servos.turnToAngle(targetAngle.getAngle());
    }
}
