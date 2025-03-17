package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static com.acmerobotics.roadrunner.Math.lerp;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MAX;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MIN;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo.LINKAGE_MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo.LINKAGE_MIN_ANGLE;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensor.ColorRangefinderEx;

@Config
public final class Intake {
    private final ServoEx[] intakeLinkGroup;

    private final MotorEx intake;


    public static double
            V4B_MIN_DOWN_ANGLE = 56,
            V4B_MAX_DOWN_ANGLE = 62,
            V4B_DOWN_ANGLE = V4B_MAX_DOWN_ANGLE,
            V4B_CLEARING_ANGLE = 85,
            V4B_UP_ANGLE = 100,
            V4B_UNSAFE_THRESHOLD_ANGLE = 0,
            V4B_TRANSFER_ANGLE = 65,
            V4B_HOVERING_ANGLE = 45;

    private V4BAngle targetAngle = V4BAngle.UP;

    private final ColorRangefinderEx rangefinder;

    public enum V4BAngle {
        DOWN,
        CLEARING,
        UP,
        UNSAFE,
        TRANSFER,
        HOVERING;

        private double getAngle() {
            switch (this) {
                case DOWN: return V4B_DOWN_ANGLE;
                case CLEARING: return V4B_CLEARING_ANGLE;
                case UNSAFE: return V4B_UNSAFE_THRESHOLD_ANGLE;
                case TRANSFER: return V4B_TRANSFER_ANGLE;
                case HOVERING: return V4B_HOVERING_ANGLE;
                default: return V4B_UP_ANGLE;
            }
        }

        public boolean isV4BUnsafe() {
            return getAngle() <= V4B_UNSAFE_THRESHOLD_ANGLE;
        }
    }

    public boolean isV4BLocked = false;

    public boolean isRollerLocked = false;

    private double rollerPower = 0;

    public Intake(HardwareMap hardwareMap) {
        intake = new MotorEx(hardwareMap, "intakeMaster");
        ServoEx intakeGearFollower = new SimpleServo(hardwareMap, "intakeLinkFollower", SERVO_25_KG_MIN, SERVO_25_KG_MAX);
        ServoEx intakeGearMaster = new SimpleServo(hardwareMap, "intakeLinkMaster", SERVO_25_KG_MIN, SERVO_25_KG_MAX);

        intakeGearMaster.setInverted(true);

        rangefinder = new ColorRangefinderEx(hardwareMap);

        intakeLinkGroup = new ServoEx[] {intakeGearFollower, intakeGearMaster};
    }

    public boolean setTargetV4BAngle(V4BAngle angle, boolean isOverride) {
        if (isV4BLocked && !isOverride) return false;
        targetAngle = angle;

        return true;
    }

    public boolean setTargetV4BAngle(V4BAngle angle) {
        return setTargetV4BAngle(angle, false);
    }

    public V4BAngle getTargetV4BAngle() {
        return targetAngle;
    }

    public boolean setRollerPower(double power, boolean isOverride) {
        if (isRollerLocked && !isOverride) return false;

        rollerPower = power;

        intake.set(rollerPower);

        return true;
    }

    public double getRollerPower() {
        return rollerPower;
    }



    public boolean setRollerPower(double power) {
        return setRollerPower(power, false);
    }

    public void run(double extendoAngle) {
        rangefinder.run();

        for (ServoEx servos : intakeLinkGroup) {
            if (getTargetV4BAngle() == V4BAngle.DOWN) {
                V4B_DOWN_ANGLE = lerp(extendoAngle, LINKAGE_MIN_ANGLE, LINKAGE_MAX_ANGLE, V4B_MIN_DOWN_ANGLE, V4B_MAX_DOWN_ANGLE);
            }

            servos.turnToAngle(targetAngle.getAngle());
        }

    }

    public ColorRangefinderEx.SampleColor getCurrentSample() {
        return rangefinder.getRawReading();
    }


    public void printTelemetry() {
        mTelemetry.addData("Sample Color", getCurrentSample());
        mTelemetry.addData("V4B State", targetAngle.name());
//        mTelemetry.addData("Raw Color", getRawColor());
    }
}
