package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class Intake {
    private final CRServo[] intakeGroup;
    private final ServoEx[] intakeLinkGroup;

    private final CRServo
        intakeFollower,
        intakeMaster;

    private final ServoEx
            intakeGearFollower,
            intakeGearMaster;

    private V4BAngles targetAngle;

    public enum V4BAngles {
        DOWN(75),
        CLEARING(45),
        UP(115);

        public final int angle;

        V4BAngles(int angle) {
            this.angle = angle;
        }
    }
    Intake(HardwareMap hardwareMap) {
        intakeFollower = new CRServo(hardwareMap, "intakeFollower");
        intakeMaster = new CRServo(hardwareMap, "intakeMaster");
        intakeGearFollower = new SimpleServo(hardwareMap, "intakeLinkFollower", 0, 180);
        intakeGearMaster = new SimpleServo(hardwareMap, "intakeLinkMaster", 0, 180);

        intakeFollower.setInverted(true);
        intakeGearFollower.setInverted(true);

        intakeGroup = new CRServo[] {intakeFollower, intakeMaster};
        intakeLinkGroup = new ServoEx[] {intakeGearFollower, intakeGearMaster};
    }

    public V4BAngles getTargetAngle() {
        return targetAngle;
    }

    public void setTarget(V4BAngles angle) {
        targetAngle = angle;
    }

    public void setServoPower(double power) {
        for (CRServo servos : intakeGroup)
            servos.set(power);
    }
    public void run() {
        for (ServoEx servos : intakeLinkGroup)
            servos.turnToAngle(targetAngle.angle);
    }
}
