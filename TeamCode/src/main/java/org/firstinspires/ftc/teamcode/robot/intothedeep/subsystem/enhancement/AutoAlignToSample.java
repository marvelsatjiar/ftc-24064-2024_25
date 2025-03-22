package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.FORWARD;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.IS_RED;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.sensor.ColorRangefinderEx;
import org.firstinspires.ftc.teamcode.sensor.vision.LimelightEx;

import java.util.List;
import java.util.TreeMap;

@Config
public class AutoAlignToSample {
    private final LimelightEx limelightEx;

    public ColorRangefinderEx.SampleColor targetColor;

    private final PIDController headingPID = new PIDController();

    public static PIDGains headingGains = new PIDGains(
            0.02125,
            0,
            0
    );

    private final TreeMap<Double, Double> estimatedExtendoAngles = new TreeMap<>();

    public static double
            tXAngle = 2;

    private LLResult desiredSample;

    public boolean
            isDrivingToSample = false,
            isSampleTargeted = false,
            isOppositeSample = false,
            isYellowSample = false;


    public AutoAlignToSample(LimelightEx limelightEx) {
        this.limelightEx = limelightEx;

        headingPID.setGains(headingGains);

        // sets hashmap keys and values
        estimatedExtendoAngles.put(0.25, 96.75);
        estimatedExtendoAngles.put(0.5, 80.625);
        estimatedExtendoAngles.put(0.75, 64.5);
        estimatedExtendoAngles.put(1.0, 48.375);
        estimatedExtendoAngles.put(1.25, 32.25);
        estimatedExtendoAngles.put(1.5, 16.125);
    }

    public void activateLimelight(int detectionPipeline) {
        targetColor = IS_RED ? ColorRangefinderEx.SampleColor.RED : ColorRangefinderEx.SampleColor.BLUE;

        limelightEx.enableStagelite(true);

        limelightEx.getLimelight().pipelineSwitch(detectionPipeline);

        limelightEx.getLimelight().setPollRateHz(100);

        limelightEx.getLimelight().start();
    }

    // add lock mechanism here (for sample)- for this you must also get current detections!!!
    public boolean targetSample() {
        LLResult result = limelightEx.update();
        limelightEx.getLimelight().reloadPipeline();
//        List<LLResultTypes.DetectorResult> targets = limelightEx.getDetectorResult();

        // checks and returns detection
        if (result != null && result.isValid()) {
            desiredSample = result;

            return true;
        }

        return false;
    }

    private Double calculateExtendoTarget() {
        if (desiredSample != null) {
            double currentArea = desiredSample.getTa();

            mTelemetry.addData("current area : ", currentArea);

            if (estimatedExtendoAngles.containsKey(currentArea)) return estimatedExtendoAngles.get(currentArea);

            // x = tArea; y = target extendo angle

            if (currentArea > estimatedExtendoAngles.firstKey() && currentArea < estimatedExtendoAngles.lastKey()) {
                double upperBound = estimatedExtendoAngles.ceilingKey(currentArea); // x2
                double lowerBound = estimatedExtendoAngles.floorKey(currentArea); // x1

                double upperValue = estimatedExtendoAngles.get(upperBound); // y2
                double lowerValue = estimatedExtendoAngles.get(lowerBound); // y1

                double interpolation = lowerValue + (currentArea - lowerBound) * ((upperValue - lowerValue) / (upperBound - lowerBound));
                mTelemetry.addData("extendo interpolation : ", interpolation);

                // linear interpolation formula
                return interpolation;
            }
        }

        return robot.extendo.getTargetAngle();
    }

    private PoseVelocity2d calculateHeadingTarget() {
        if (desiredSample != null) {
            double theta = desiredSample.getTx();

            mTelemetry.addData(" current x degrees : ", theta);

            State currentHeading = new State(theta);

            double headingPower = headingPID.calculate(currentHeading);

            mTelemetry.addData("heading power : ", headingPower);

            return new PoseVelocity2d(
                    new Vector2d(
                            0,
                            0
                    ),
                    headingPower
            );
        }

        return new PoseVelocity2d(
                new Vector2d(
                        0,
                        0
                ),
                0
        );
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

    public Action driveToTarget(double secondsToExpire) {
        return new Action() {
            boolean isFirstTime = true;
            final ElapsedTime expirationTimer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isFirstTime) {
                    isFirstTime = false;
                    expirationTimer.startTime();
                    robot.intake.setTargetV4BAngle(Intake.V4BAngle.DOWN);

                    headingPID.setTarget(new State(tXAngle));
                }

                // update detections
                isSampleTargeted = targetSample();

                if (isSampleTargeted) {
                    // send out output to motors/servos
                    robot.extendo.setTargetAngle(calculateExtendoTarget(), true);
                    robot.drivetrain.setFieldCentricPowers(calculateHeadingTarget());

                    robot.drivetrain.updatePoseEstimate();
//                setEdgeCases();
                }

                mTelemetry.addData("is sample targeted? ", isSampleTargeted);
                mTelemetry.addData("is expired? ", expirationTimer.seconds() > secondsToExpire);

                return expirationTimer.seconds() <= secondsToExpire;
            }
        };
    }
}
