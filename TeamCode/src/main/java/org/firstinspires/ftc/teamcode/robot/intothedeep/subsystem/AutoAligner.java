package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.sensor.DistanceSensorEx;

public class AutoAligner {
    private final DistanceSensorEx
            leftDistanceSensor,
            rightDistanceSensor;

    private final PIDController xyPIDController = new PIDController();
    private final PIDController headingPIDController = new PIDController();

    private final PIDGains xyPIDGains = new PIDGains(
            0,
            0,
            0
    );

    private final PIDGains headingPIDGains = new PIDGains(
            0,
            0,
            0
    );

    private static final State
            SUBMERSIBLE_STATE = new State(3),
            WALL_PICKUP_STATE = new State(1),
            CLIMB_STATE = new State(2);

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

    public static final double SENSOR_DISTANCE = 3;

    private TargetDistance targetDistance = TargetDistance.INACTIVE;

    private State currentXYState = new State(0);
    private State currentHeadingState = new State(0);

    AutoAligner(HardwareMap hardwareMap) {
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
        return (headingPIDController.isPositionInTolerance(currentHeadingState, 0.174533) && xyPIDController.isPositionInTolerance(currentXYState, 0.5));
    }

    public PoseVelocity2d run(double y) {
        double theta = Math.atan2((rightDistanceSensor.calculateDistance() - leftDistanceSensor.calculateDistance()), SENSOR_DISTANCE);
        currentXYState = new State(Math.min(rightDistanceSensor.calculateDistance(), leftDistanceSensor.calculateDistance()) * Math.cos(theta));
        currentHeadingState = new State(theta);

        return new PoseVelocity2d(
                new Vector2d(
                        getXYDistance(),
                        y
                ),
                getHeadingDistance()
        );
    }

}
