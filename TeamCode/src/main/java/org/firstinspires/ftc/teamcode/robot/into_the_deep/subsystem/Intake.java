package org.firstinspires.ftc.teamcode.robot.into_the_deep.subsystem;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class Intake {
    private int targetAngle;
    private final CRServo[] intakeGroup;
    private final ServoEx[] intakeLinkGroup;

    public enum IntakeAngles {
        DOWN,
        CLEARING,
        UP
    }
    private IntakeAngles targetState;
    Intake(HardwareMap hardwareMap) {
        CRServo intakeFollower = new CRServo(hardwareMap, "intakeFollower");
        CRServo intakeMaster = new CRServo(hardwareMap, "intakeMaster");
        ServoEx intakeLinkFollower = new SimpleServo(hardwareMap, "intakeLinkFollower", 0, 180);
        ServoEx intakeLinkMaster = new SimpleServo(hardwareMap, "intakeLinkMaster", 0, 180);

        intakeFollower.setInverted(true);
        intakeLinkFollower.setInverted(true);

        intakeGroup = new CRServo[] {intakeFollower, intakeMaster};
        intakeLinkGroup = new ServoEx[] {intakeLinkFollower, intakeLinkMaster};
    }

    public void setAngle() {
        switch (targetState) {
            case DOWN:
                targetAngle = 75;
                break;
            case UP:
                targetAngle = 115;
                break;
            case CLEARING:
                targetAngle = 45;
                break;
        }
    }

    public Intake.IntakeAngles getSetPoint() {
        return targetState;
    }

    public void setTargetPoint(int i) {
        switch (i) {
            case 1:
                targetState = IntakeAngles.CLEARING;
                break;
            case 2:
                targetState = IntakeAngles.UP;
                break;
            case 3:
                targetState = IntakeAngles.DOWN;
                break;
        }
    }

    public void setServoPower(double power) {for (CRServo servos : intakeGroup) servos.set(power);}
    public void run() {
        setAngle();
        for (ServoEx servos : intakeLinkGroup) servos.turnToAngle(targetAngle);
    }
}
