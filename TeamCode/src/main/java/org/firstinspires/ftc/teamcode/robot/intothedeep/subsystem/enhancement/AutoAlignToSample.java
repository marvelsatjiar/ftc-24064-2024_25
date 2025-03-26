package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.IS_RED;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.sensor.ColorRangefinderEx;
import org.firstinspires.ftc.teamcode.sensor.vision.LimelightEx;

import java.util.List;
import java.util.TreeMap;

@Config
public class AutoAlignToSample {
    private final LimelightEx limelightEx;

    public ColorRangefinderEx.SampleColor targetColor;

    private final TreeMap<Double, Double> estimatedExtendoAngles = new TreeMap<>();

    public static double
            xOffset = -4,
            yOffset = -1,
            limelightTilt = 60,
            limelightCrosshairDistance = 60,
            limelightHeight = 11;

    public static double secondsUntilCollected = 3;

    private LLResultTypes.DetectorResult desiredSample;

    private Action targetSampleTrajectory;

    private boolean
            isSampleDetected = false,
            isTurningOnly = false;

    public boolean
            isOppositeSample = false,
            isYellowSample = false;

    private double targetedExtendoAngle = 0;

    private Pose2d targetedPoseOffset = new Pose2d(0, 0, 0);

    public AutoAlignToSample(LimelightEx limelightEx) {
        this.limelightEx = limelightEx;

        // sets hashmap keys and values
        estimatedExtendoAngles.put(7.0, 96.75);
        estimatedExtendoAngles.put(3.0, 80.625);
        estimatedExtendoAngles.put(0.0, 64.5);
        estimatedExtendoAngles.put(-3., 48.375);
        estimatedExtendoAngles.put(-6.0, 32.25);
        estimatedExtendoAngles.put(-14.5, 16.125);
    }

    public void activateLimelight(int detectionPipeline) {
        targetColor = IS_RED ? ColorRangefinderEx.SampleColor.RED : ColorRangefinderEx.SampleColor.BLUE;

        limelightEx.enableStagelite(true);

        limelightEx.getLimelight().pipelineSwitch(detectionPipeline);

        limelightEx.getLimelight().setPollRateHz(10);
    }

    // add lock mechanism here (for sample)- for this you must also get current detections!!!
    public boolean targetSample() {
        limelightEx.update();
        List<LLResultTypes.DetectorResult> targets = limelightEx.getDetectorResult();

        // checks and returns detection
        if (targets != null && !targets.isEmpty()) {
            for (LLResultTypes.DetectorResult result : targets) {
                if (desiredSample == null || result.getTargetYDegrees() - result.getTargetArea() > desiredSample.getTargetYDegrees() - desiredSample.getTargetArea()) {
                    desiredSample = result;
                }
            }

            return true;
        }

        return false;
    }

    private double calculateExtendoTarget() {
        if (desiredSample != null) {
            double yDistance = Math.tan(Math.atan(limelightCrosshairDistance/limelightHeight) + desiredSample.getTargetYDegrees()) * limelightHeight;

            yDistance += yOffset;

            if (estimatedExtendoAngles.containsKey(yDistance)) return estimatedExtendoAngles.get(yDistance);

            // x = tArea; y = target extendo angle

            if (yDistance >= estimatedExtendoAngles.firstKey() && yDistance <= estimatedExtendoAngles.lastKey()) {
                double upperBound = estimatedExtendoAngles.ceilingKey(yDistance); // x2
                double lowerBound = estimatedExtendoAngles.floorKey(yDistance); // x1

                double upperValue = estimatedExtendoAngles.get(upperBound); // y2
                double lowerValue = estimatedExtendoAngles.get(lowerBound); // y1

                // y = y1 + ((x-x1)(y2-y1))/(x2-x1)
                double interpolation = lowerValue + ((yDistance - lowerBound) * (upperValue - lowerValue)) / (upperBound - lowerBound);
                mTelemetry.addData("extendo interpolation : ", interpolation);

                // linear interpolation formula
                return interpolation;
            }
        }

        return robot.extendo.getTargetAngle();
    }

    private Pose2d calculateTargetPosition(boolean isTurning) {
        double yDistance = Math.tan(Math.toRadians(limelightTilt + desiredSample.getTargetYDegrees())) * limelightHeight;
        double xDistance = Math.tan(Math.toRadians(desiredSample.getTargetXDegrees())) * yDistance + xOffset;

        yDistance += yOffset;

        double headingDistance = Math.atan2(xDistance + xOffset, yDistance + yOffset);

        mTelemetry.addData(" x distance : ", xDistance);
        mTelemetry.addData(" y distance : ", yDistance);
        mTelemetry.addData(" heading distance : ", headingDistance);

        if (isTurning) {
            isTurningOnly = true;
            return new Pose2d(0, 0, headingDistance);
        } else {
            return new Pose2d(xDistance, 0, 0); // should be xDistance TODO
        }
    }

//    private ColorRangefinderEx.SampleColor getTargetColor() {
//        if (desiredSample != null) {
//            switch (desiredSample.getClassId()) {
//                case 0:
//                    return ColorRangefinderEx.SampleColor.BLUE;
//                case 1:
//                    return ColorRangefinderEx.SampleColor.RED;
//                case 2:
//                    return ColorRangefinderEx.SampleColor.YELLOW;
//            }
//        }
//        return ColorRangefinderEx.SampleColor.NOTHING;
//    }

//    private void setEdgeCases() {
//        boolean ifBlueSample = robot.intake.getCurrentSample() == ColorRangefinderEx.SampleColor.BLUE;
//        boolean ifRedSample = robot.intake.getCurrentSample() == ColorRangefinderEx.SampleColor.RED;
//        boolean ifYellowSample = robot.intake.getCurrentSample() == ColorRangefinderEx.SampleColor.YELLOW;
//
//        switch (targetColor) {
//            case RED: {
//                isOppositeSample = ifBlueSample;
//                isYellowSample = ifYellowSample;
//                break;
//            }
//            case BLUE: {
//                isOppositeSample = ifRedSample;
//                isYellowSample = ifYellowSample;
//                break;
//            }
//            case YELLOW: {
//                isOppositeSample = ifBlueSample || ifRedSample;
//                break;
//            }
//        }
//    }

    public Action detectTarget(double secondsToExpire, boolean isTurning) {
        return new Action() {
            boolean isFirstTime = true;
            final ElapsedTime expirationTimer = new ElapsedTime();

            @Override
            public boolean run(TelemetryPacket telemetryPacket) {
                if (isFirstTime) {
                    isFirstTime = false;
                    expirationTimer.reset();
                }

                if (!isSampleDetected) {
                    isSampleDetected = targetSample();
                }

                if (isSampleDetected) {
                    // send out output to motors/servos
                    targetedPoseOffset = calculateTargetPosition(isTurning);
                    targetedExtendoAngle = calculateExtendoTarget();
//                  setEdgeCases();
                }

                mTelemetry.addData("is sample targeted? ", isSampleDetected);
                mTelemetry.addData("is expired? ", expirationTimer.seconds() > secondsToExpire);

                return expirationTimer.seconds() <= secondsToExpire && !isSampleDetected;
            }
        };
    }

    public void generateTargetTrajectory() {
        if (!isTurningOnly) {
            targetSampleTrajectory = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                    .lineToX(robot.drivetrain.pose.position.x + targetedPoseOffset.position.x)
                    .afterTime(0, new ParallelAction(
                            RobotActions.runRollersUntilCollected(1, targetColor, secondsUntilCollected),
                            RobotActions.setExtendo(targetedExtendoAngle, 0),
                            RobotActions.setV4B(Intake.V4BAngle.DOWN, 0)
                    ))
                    .build();
        } else {
            targetSampleTrajectory = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                    .turn(robot.drivetrain.pose.heading.toDouble() + targetedPoseOffset.heading.toDouble())
                    .afterTime(0, new ParallelAction(
                            RobotActions.runRollersUntilCollected(1, targetColor, secondsUntilCollected),
                            RobotActions.setV4B(Intake.V4BAngle.DOWN, 0)
                    ))
                    .build();
        }

    }

    public boolean wasSampleDetected() {
        return isSampleDetected;
    }

    public Action getTargetSampleTrajectory() {
        return targetSampleTrajectory;
    }
}
