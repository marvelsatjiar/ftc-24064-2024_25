package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.Actions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Sweeper;
import org.firstinspires.ftc.teamcode.sensor.ColorRangefinderEx;
import org.firstinspires.ftc.teamcode.sensor.vision.LimelightEx;

import java.util.List;

@Config
public class AutoAlignToSample {
    private final LimelightEx limelightEx;

    public ColorRangefinderEx.SampleColor targetColor;

    public static double
            xOffset = 5,
            yOffset = 2,
            sampleOffset = 6.5,
            limelightTilt = 35,
            limelightHeight = 11;

    public static double secondsUntilCollected = 0.6;

    private LLResultTypes.DetectorResult desiredSample;

    private Action targetSampleTrajectory;
    
    public static class AutoAlign {
        public double
                sleepSecondsBeforeV4bUp = 0.4,
                sleepSecondsBeforeV4bDown = 0.2,
                sleepSecondsUntilDesiredExtension = 1,
                sleepSecondsBeforeMoving = 0.5,
                extensionOffset = 45,
                sleepSecondsBeforeRollersDeactivate = 0.2;
    }

    private boolean
            isSampleDetected = false,
            isTurningOnly = false;

    private double
            xDistance = 0,
            yDistance = 0,
            headingDistance = 0;

    private double targetedExtendoAngle = 0;

    public static AutoAlign A_A = new AutoAlign();

    private Pose2d targetedPoseOffset = new Pose2d(0, 0, 0);

    public AutoAlignToSample(LimelightEx limelightEx) {
        this.limelightEx = limelightEx;
    }

    public void activateLimelight(int detectionPipeline, ColorRangefinderEx.SampleColor color) {
        targetColor = color;

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
            
            if (isTurningOnly) finalDistance = Math.sqrt((yDistance-sampleOffset)*(yDistance-sampleOffset) + xDistance*xDistance);
            else finalDistance = (yDistance-sampleOffset);

            return robot.extendo.convertTargetInchesToExtensionAngle(finalDistance);
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
            return new Pose2d(0, xDistance, 0); // should be xDistance TODO
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

    public Action detectTarget(double secondsToExpire, boolean isTurning) {
        return new Action() {
            boolean isFirstTime = true;
            final ElapsedTime expirationTimer = new ElapsedTime();

            @Override
            public boolean run(TelemetryPacket telemetryPacket) {
                if (isFirstTime) {
                    isSampleDetected = false;
                    isFirstTime = false;
                    limelightEx.getLimelight().captureSnapshot("detection");
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
        if (isSampleDetected) {
            if (!isTurningOnly) {
                targetSampleTrajectory = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                        .afterTime(0, new ParallelAction(
                                RobotActions.setExtendo(targetedExtendoAngle, 0),
                                new SequentialAction(
                                        RobotActions.setSweeper(Sweeper.SweeperAngles.AUTON_ACTIVE, 1),
                                        RobotActions.setSweeper(Sweeper.SweeperAngles.RETRACTED, 0)
                                )
                        ))

                        .strafeTo(new Vector2d(robot.drivetrain.pose.position.x + targetedPoseOffset.position.x, robot.drivetrain.pose.position.y + targetedPoseOffset.position.y))
                        .afterTime(A_A.sleepSecondsBeforeV4bDown, RobotActions.setV4B(Intake.V4BAngle.DOWN, 0))
                        .waitSeconds(A_A.sleepSecondsBeforeMoving)
                        .stopAndAdd(RobotActions.runRollersUntilCollected(0.8, targetColor, secondsUntilCollected))

                        .stopAndAdd(RobotActions.setExtendo(targetedExtendoAngle + A_A.extensionOffset, A_A.sleepSecondsUntilDesiredExtension))
                        .build();
            } else {
                targetSampleTrajectory = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                        .afterTime(0, RobotActions.setExtendo(targetedExtendoAngle, 0))
                        .turn(-targetedPoseOffset.heading.toDouble())
                        .afterTime(A_A.sleepSecondsBeforeV4bDown, RobotActions.setV4B(Intake.V4BAngle.DOWN, 0))
                        .waitSeconds(A_A.sleepSecondsBeforeMoving)
                        .stopAndAdd(RobotActions.runRollersUntilCollected(0.8, targetColor, secondsUntilCollected))
//                        .stopAndAdd(this::setFullExtensionIfNotCollectedSpecimenSide)
                        .stopAndAdd(RobotActions.setExtendo(targetedExtendoAngle + A_A.extensionOffset, A_A.sleepSecondsUntilDesiredExtension))
                        .build();
            }
        } else {
            targetSampleTrajectory = new NullAction();
        }
    }

    public Action setFullExtensionIfNotCollectedSampleSide() {
        if (!robot.intake.isCorrectSample()) {
            return new SequentialAction(
                    RobotActions.setExtendo(Extendo.Extension.EXTENDED, A_A.sleepSecondsBeforeV4bUp),
                    RobotActions.setV4B(Intake.V4BAngle.UP, A_A.sleepSecondsBeforeRollersDeactivate),
                    RobotActions.retractForTransfer()
            );
        } else {
            return RobotActions.retractForTransfer();
        }
    }

    public Action setFullExtensionIfNotCollectedSpecimenSide() {
        if (!robot.intake.isCorrectSample()) {
            return new SequentialAction(
                    RobotActions.setExtendo(Extendo.Extension.EXTENDED, A_A.sleepSecondsBeforeV4bUp),
                    RobotActions.setV4B(Intake.V4BAngle.UP, A_A.sleepSecondsBeforeRollersDeactivate),
                    RobotActions.retractExtendo()
            );
        } else {
            return RobotActions.retractExtendo();
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
