package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MAX;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.SERVO_25_KG_MIN;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensor.ColorRangefinderEx;

@Config
public final class Intake {
    private final CRServo[] intakeGroup;
    private final ServoEx[] intakeLinkGroup;

    public static int
            V4B_DOWN_ANGLE = 107,
            V4B_FRONT_WALL_PICKUP_ANGLE = 35,
            V4B_CLEARING_ANGLE = 85,
            V4B_UP_ANGLE = 69,
            V4B_UNSAFE_THRESHOLD_ANGLE = 108,
            V4B_TRANSFER_ANGLE = 30,
            V4B_HOVERING_ANGLE = 45;

    private V4BAngle targetAngle = V4BAngle.UP;

    private final ColorRangefinderEx rangefinder;


    public enum V4BAngle {
        DOWN,
        FRONT_WALL_PICKUP,
        CLEARING,
        UP,
        UNSAFE,
        TRANSFER,
        HOVERING;

        private int getAngle() {
            switch (this) {
                case DOWN: return  V4B_DOWN_ANGLE;
                case FRONT_WALL_PICKUP: return V4B_FRONT_WALL_PICKUP_ANGLE;
                case CLEARING: return V4B_CLEARING_ANGLE;
                case UNSAFE: return V4B_UNSAFE_THRESHOLD_ANGLE;
                case TRANSFER: return V4B_TRANSFER_ANGLE;
                case HOVERING: return V4B_HOVERING_ANGLE;
                default: return V4B_UP_ANGLE;
            }
        }

        public boolean isV4BUnsafe() {
            return getAngle() >= V4B_UNSAFE_THRESHOLD_ANGLE;
        }
    }

    public boolean isV4BLocked = false;

    public boolean isRollerLocked = false;

    private double rollerPower = 0;

    public Intake(HardwareMap hardwareMap) {
        CRServo intakeFollower = new CRServo(hardwareMap, "intakeFollower");
        CRServo intakeMaster = new CRServo(hardwareMap, "intakeMaster");
        ServoEx intakeGearFollower = new SimpleServo(hardwareMap, "intakeLinkFollower", SERVO_25_KG_MIN, SERVO_25_KG_MAX);
        ServoEx intakeGearMaster = new SimpleServo(hardwareMap, "intakeLinkMaster", SERVO_25_KG_MIN, SERVO_25_KG_MAX);

        intakeFollower.setInverted(true);
        intakeGearMaster.setInverted(true);

        rangefinder = new ColorRangefinderEx(hardwareMap);

        intakeGroup = new CRServo[] {intakeFollower, intakeMaster};
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

        for (CRServo servos : intakeGroup)
            servos.set(power);

        return true;
    }

    public double getRollerPower() {
        return rollerPower;
    }



    public boolean setRollerPower(double power) {
        return setRollerPower(power, false);
    }

    public void run() {
        rangefinder.run();

        for (ServoEx servos : intakeLinkGroup)
            servos.turnToAngle(targetAngle.getAngle());
    }

    public ColorRangefinderEx.SampleColor getCurrentSample() {
        return rangefinder.getRawReading();
    }
//    public ColorRangefinderEx.SampleColor getRawColor(){
//        return rangefinder.getRawReading();
//    }


    public void printTelemetry() {
        mTelemetry.addData("Sample Color", getCurrentSample());
        mTelemetry.addData("V4B State", targetAngle.name());
//        mTelemetry.addData("Raw Color", getRawColor());
    }
}
