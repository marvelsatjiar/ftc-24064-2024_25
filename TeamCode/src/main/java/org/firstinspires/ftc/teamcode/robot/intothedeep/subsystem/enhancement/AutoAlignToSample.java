package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.IS_RED;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
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
            limelightHeight = 11,
            limelightCrosshairDistance = 20;

    public static double secondsUntilCollected = 3;

    private LLResultTypes.DetectorResult desiredSample;

    private boolean isSampleDetected = false;

    public boolean
            isOppositeSample = false,
            isYellowSample = false;

    private Pose2d targetedPoseOffset = new Pose2d(0, 0, 0);

    public AutoAlignToSample(LimelightEx limelightEx) {
        this.limelightEx = limelightEx;

        // sets hashmap keys and values
        estimatedExtendoAngles.put(4.0, 96.75);
        estimatedExtendoAngles.put(3.5, 80.625);
        estimatedExtendoAngles.put(3.0, 64.5);
        estimatedExtendoAngles.put(2.5, 48.375);
        estimatedExtendoAngles.put(2.0, 32.25);
        estimatedExtendoAngles.put(1.5, 16.125);
    }

    public void activateLimelight(int detectionPipeline) {
        targetColor = IS_RED ? ColorRangefinderEx.SampleColor.RED : ColorRangefinderEx.SampleColor.BLUE;

        limelightEx.enableStagelite(true);

        limelightEx.getLimelight().pipelineSwitch(detectionPipeline);

        limelightEx.getLimelight().setPollRateHz(10);
    }

    // add lock mechanism here (for sample)- for this you must also get current detections!!!
    public boolean targetSample() {
        List<LLResultTypes.DetectorResult> targets = limelightEx.getDetectorResult();

        // checks and returns detection
        if (targets != null && !targets.isEmpty()) {
            for (LLResultTypes.DetectorResult result : targets) {
                if (desiredSample == null || result.getTargetArea() > desiredSample.getTargetArea()) {
                    desiredSample = result;
                }
            }

            return true;
        }

        return false;
    }

    private Double calculateExtendoTarget() {
        if (desiredSample != null) {
            double yDistance = Math.tan(Math.atan(limelightCrosshairDistance/limelightHeight) - desiredSample.getTargetYDegrees()) * limelightHeight;;

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

    private Pose2d calculateTargetPosition() {
        double yDistance = Math.tan(Math.atan(limelightCrosshairDistance/limelightHeight) - desiredSample.getTargetYDegrees()) * limelightHeight;
        double xDistance = Math.tan(desiredSample.getTargetXDegrees()) * yDistance;

        double headingDistance = Math.atan2(yDistance, xDistance);

        mTelemetry.addData(" x distance : ", xDistance);
        mTelemetry.addData(" y distance : ", yDistance);
        mTelemetry.addData(" heading distance : ", headingDistance);

        return new Pose2d(xDistance, yDistance, headingDistance);
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

    public Action detectTarget(double secondsToExpire) {
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
                    robot.extendo.setTargetAngle(calculateExtendoTarget(), true);
                    targetedPoseOffset = calculateTargetPosition();
//                  setEdgeCases();
                }

                mTelemetry.addData("is sample targeted? ", isSampleDetected);
                mTelemetry.addData("is expired? ", expirationTimer.seconds() > secondsToExpire);

                return expirationTimer.seconds() <= secondsToExpire || !isSampleDetected;
            }
        };
    }

    public void driveToTarget() {
        Actions.runBlocking(robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                .strafeToLinearHeading(
                    new Vector2d(
                            robot.drivetrain.pose.position.x + targetedPoseOffset.position.x,
                            robot.drivetrain.pose.position.y + targetedPoseOffset.position.y
                    ),
                    robot.drivetrain.pose.heading.toDouble() + targetedPoseOffset.heading.toDouble()
                )
                .afterTime(0, new ParallelAction(
                        RobotActions.runRollersUntilCollected(1, targetColor, secondsUntilCollected),
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0)
                ))
                .build()
        );
    }

    public boolean wasSampleDetected() {
        return isSampleDetected;
    }
}
