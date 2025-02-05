package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.motion.State;

public final class Common {
    public static Pose2d AUTO_END_POSE = null;
    public static boolean IS_RED = false, IS_SPECIMEN_SIDE = false;

    public static final double
            LEFT = Math.toRadians(180),
            FORWARD = Math.toRadians(90),
            RIGHT = Math.toRadians(0),
            BACKWARD = Math.toRadians(270),
            SERVO_25_KG_MIN = 0,
            SERVO_25_KG_MAX = 270,
            SERVO_45_KG_MIN = 0,
            SERVO_45_KG_MAX = 270,
            SERVO_AXON_MIN = 0,
            SERVO_AXON_MAX = 255;

    public static final int LIMELIGHT_SPECIMEN_NN_PIPELINE = 4;

    public static final double MAX_VOLTAGE = 13;

    public static Robot robot;
    public static MultipleTelemetry mTelemetry;

}
