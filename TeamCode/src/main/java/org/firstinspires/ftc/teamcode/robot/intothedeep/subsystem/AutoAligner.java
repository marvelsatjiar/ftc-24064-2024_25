package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.sensor.DistanceSensorEx;

@Config
public class AutoAligner {
    private final DistanceSensorEx
            leftDistanceSensor,
            rightDistanceSensor;

    public static double
            headingTolerance = 10,
            xyTolerance = 0.5;

    private final PIDController xyPIDController = new PIDController();
    private final PIDController headingPIDController = new PIDController();

    public static PIDGains xyPIDGains = new PIDGains(
            0.0525,
            0,
            0.000002,
            Double.POSITIVE_INFINITY
    );

    public static PIDGains headingPIDGains = new PIDGains(
            0.4,
            0,
            0.0000000125,
            Double.POSITIVE_INFINITY
    );

    public static double
        SUBMERSIBLE_TARGET = 4,
        WALL_PICKUP_TARGET = 1,
        CLIMB_TARGET = 2;

    private static State
            SUBMERSIBLE_STATE = new State(SUBMERSIBLE_TARGET),
            WALL_PICKUP_STATE = new State(WALL_PICKUP_TARGET),
            CLIMB_STATE = new State(CLIMB_TARGET);

    public enum TargetDistance {
        SUBMERSIBLE,
        WALL_PICKUP,
        CLIMB,
        INACTIVE;

        private State toState() {
            switch (this) {
                case SUBMERSIBLE: return SUBMERSIBLE_STATE;
                case WALL_PICKUP: return WALL_PICKUP_STATE;
                case CLIMB: return CLIMB_STATE;
                case INACTIVE: default: return null;
            }
        }
    }

    public static final double SENSOR_DISTANCE = 2.5625;

    private TargetDistance targetDistance = TargetDistance.INACTIVE;

    private State currentXYState = new State(0);
    private State currentHeadingState = new State(0);

    public AutoAligner(HardwareMap hardwareMap) {
        xyPIDController.setGains(xyPIDGains);
        headingPIDController.setGains(headingPIDGains);
        headingPIDController.setTarget(new State(0));

        leftDistanceSensor = new DistanceSensorEx(hardwareMap.get(DistanceSensor.class, "left distance"));
        rightDistanceSensor = new DistanceSensorEx(hardwareMap.get(DistanceSensor.class, "right distance"));
    }

    public TargetDistance getTargetDistance() {
        return targetDistance;
    }

    public void setTargetDistance(TargetDistance distance) {
        targetDistance = distance;
        xyPIDController.setTarget(distance.toState());
    }

    private double getXYDistance() {
        return xyPIDController.calculate(currentXYState);
    }

    private double getHeadingDistance() {
        return headingPIDController.calculate(currentHeadingState);
    }

    public boolean isPositionInTolerance() {
        return (headingPIDController.isPositionInTolerance(currentHeadingState, Math.toRadians(headingTolerance)) && xyPIDController.isPositionInTolerance(currentXYState, xyTolerance));
    }

    public PoseVelocity2d run(double y) {
        xyPIDController.setGains(xyPIDGains);
        headingPIDController.setGains(headingPIDGains);

        double leftCalculatedDistance = leftDistanceSensor.calculateDistance();
        double rightCalculatedDistance = rightDistanceSensor.calculateDistance();

        double theta = Math.atan2(rightCalculatedDistance - leftCalculatedDistance, SENSOR_DISTANCE);
        currentXYState = new State(Math.min(rightCalculatedDistance, leftCalculatedDistance) * Math.cos(theta));
        currentHeadingState = new State(theta);

        return new PoseVelocity2d(
                new Vector2d(
                        getXYDistance(),
                        y
                ),
                getHeadingDistance()
        );
    }

    public void printTelemetry() {
        mTelemetry.addData("current xy", currentXYState.x);
        mTelemetry.addData("current heading", Math.toDegrees(currentHeadingState.x));
        mTelemetry.addData("target xy", (getTargetDistance().toState() == null ? "no target" : getTargetDistance().toState().x));
        mTelemetry.addData("target heading", 0);
        mTelemetry.addData("target", targetDistance.name());
    }

}
