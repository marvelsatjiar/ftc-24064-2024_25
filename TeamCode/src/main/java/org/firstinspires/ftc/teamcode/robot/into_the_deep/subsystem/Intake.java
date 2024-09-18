package org.firstinspires.ftc.teamcode.robot.into_the_deep.subsystem;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class Intake {
    private int targetAngle;
    private final CRServo[] intakeGroup;
    private final ServoEx[] intakeLinkGroup;

    private final CRServo
        intakeFollower,
        intakeMaster;

    private final ServoEx
            intakeLinkFollower,
            intakeLinkMaster;

    public enum IntakeAngles {
        DOWN(75),
        CLEARING(45),
        UP(115);

        public final int angles;
        IntakeAngles(int angles) {
            this.angles = angles;
        }
    }
    Intake(HardwareMap hardwareMap) {
        intakeFollower = new CRServo(hardwareMap, "intakeFollower");
        intakeMaster = new CRServo(hardwareMap, "intakeMaster");
        intakeLinkFollower = new SimpleServo(hardwareMap, "intakeLinkFollower", 0, 180);
        intakeLinkMaster = new SimpleServo(hardwareMap, "intakeLinkMaster", 0, 180);

        intakeFollower.setInverted(true);
        intakeLinkFollower.setInverted(true);

        intakeGroup = new CRServo[] {intakeFollower, intakeMaster};
        intakeLinkGroup = new ServoEx[] {intakeLinkFollower, intakeLinkMaster};
    }

    public int getSetPoint() {
        return targetAngle;
    }

    public void setTargetPoint(int i) {
        switch (i) {
            case 1:
                targetAngle = IntakeAngles.CLEARING.angles;
                break;
            case 2:
                targetAngle = IntakeAngles.UP.angles;
                break;
            case 3:
                targetAngle = IntakeAngles.DOWN.angles;
                break;
        }
    }

    public void setServoPower(double power) {
        for (CRServo servos : intakeGroup)
            servos.set(power);
    }
    public void run() {
        for (ServoEx servos : intakeLinkGroup)
            servos.turnToAngle(targetAngle);
    }
}
