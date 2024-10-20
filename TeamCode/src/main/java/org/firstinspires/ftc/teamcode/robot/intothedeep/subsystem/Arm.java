package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class Arm {
    private final ServoEx wrist;

    private final ServoEx[] armServos;

    public enum ArmAngles {
        ANGLE_COLLECTING(100, 10),
        ANGLE_DEPOSITING(60, 50),
        ANGLE_HIGH_RUNG(160, 200);

        public final int armAngle, wristAngle;

        ArmAngles(int armAngle, int wristAngle) {
            this.wristAngle = wristAngle;
            this.armAngle = armAngle;
        }
    }

    private static ArmAngles targetAngle;

    Arm(HardwareMap hardwareMap) {
        wrist = new SimpleServo(hardwareMap, "wrist", 0, 180);
        armServos = new ServoEx[]{
                new SimpleServo(hardwareMap, "arm master", 0, 240),
                new SimpleServo(hardwareMap, "arm follower", 0, 240)
        };

        armServos[1].setInverted(true);
    }

    public void setArmTarget(Arm.ArmAngles angle) {
        targetAngle = angle;
    }

    public void run() {
        for (ServoEx servos : armServos) {
            servos.turnToAngle(targetAngle.armAngle);
        }
        wrist.turnToAngle(targetAngle.wristAngle);
    }
}
