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
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.Actions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
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
            xOffset = 4,
            yOffset = 2,
            extendoOffset = 5,
            limelightTilt = 55,
            linkageLength = 10.75,
            limelightHeight = 11;

    public static double secondsUntilCollected = 3;

    private LLResultTypes.DetectorResult desiredSample;

    private Action targetSampleTrajectory;

    private boolean
            isSampleDetected = false,
            isTurningOnly = false;

    private double
            xDistance = 0,
            yDistance = 0,
            headingDistance = 0;

    public boolean
            isOppositeSample = false,
            isYellowSample = false;

    private double targetedExtendoAngle = 0;

    private Pose2d targetedPoseOffset = new Pose2d(0, 0, 0);

    public AutoAlignToSample(LimelightEx limelightEx) {
        this.limelightEx = limelightEx;

        // sets hashmap keys and values
        estimatedExtendoAngles.put(24.5, 142.0);
        estimatedExtendoAngles.put(21.1, 130.0);
        estimatedExtendoAngles.put(17.7, 108.1);
        estimatedExtendoAngles.put(13.3, 83.4);
        estimatedExtendoAngles.put(9.9, 66.75);
        estimatedExtendoAngles.put(6.5, 13.0);
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
            double finalDistance;
            
            if (isTurningOnly) finalDistance = Math.sqrt(yDistance*yDistance + xDistance*xDistance);
            else finalDistance = yDistance;

            double height = linkageLength - finalDistance;

            return Range.clip(13, (Math.atan2((finalDistance/2), height) * (180/Math.PI)) + (13 - extendoOffset), 142);
        }

        return robot.extendo.getTargetAngle();
    }

    private Pose2d calculateTargetPosition(boolean isTurning) {
        yDistance += yOffset;
        xDistance += xOffset;
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
                    yDistance = Math.tan(Math.toRadians(limelightTilt + desiredSample.getTargetYDegrees())) * limelightHeight;
                    xDistance = Math.tan(Math.toRadians(desiredSample.getTargetXDegrees())) * yDistance;

                    headingDistance = Math.atan2(xDistance + xOffset, yDistance + yOffset);

                    // send out output to motors/servos
                    targetedPoseOffset = calculateTargetPosition(isTurning);
                    targetedExtendoAngle = calculateExtendoTarget();
//                  setEdgeCases();
                }

                mTelemetry.addData("is sample targeted? ", isSampleDetected);
                mTelemetry.addData("is expired? ", expirationTimer.seconds() > secondsToExpire);

                mTelemetry.update();

                return expirationTimer.seconds() <= secondsToExpire && !isSampleDetected;
            }
        };
    }

    public void generateTargetTrajectory() {
        if (!isTurningOnly) {
            targetSampleTrajectory = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                    .strafeTo(new Vector2d(robot.drivetrain.pose.position.x + targetedPoseOffset.position.x, robot.drivetrain.pose.position.y + targetedPoseOffset.position.y))
                    .afterTime(0, new ParallelAction(
                            RobotActions.runRollersUntilCollected(0.8, targetColor, secondsUntilCollected),
                            RobotActions.setExtendo(targetedExtendoAngle, 0),
                            RobotActions.setV4B(Intake.V4BAngle.DOWN, 0)

                    ))
                    .build();
        } else {
            targetSampleTrajectory = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                    .turn(-targetedPoseOffset.heading.toDouble())

                    .afterTime(0, new ParallelAction(
                            RobotActions.runRollersUntilCollected(0.8, targetColor, secondsUntilCollected),
                            RobotActions.setExtendo(targetedExtendoAngle, 0.3),
                            RobotActions.setV4B(Intake.V4BAngle.DOWN, 0)

                    ))
                    .build();
        }

    }

    public Action updateTelemetry(boolean isOpModeActive) {
        return new Actions.RunnableAction(
                () -> {
                    mTelemetry.addData("y distance : ", yDistance);
                    mTelemetry.addData("x distance : ", xDistance);
                    mTelemetry.addData("heading distance : ", headingDistance);
                    mTelemetry.addData("extendo distance : ", targetedExtendoAngle);
                    return isOpModeActive;
                }
        );
    }

    public boolean wasSampleDetected() {
        return isSampleDetected;
    }

    public Action getTargetSampleTrajectory() {
        return targetSampleTrajectory;
    }
}
