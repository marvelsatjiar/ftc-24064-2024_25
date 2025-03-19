package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.IS_RED;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
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
            1.15,
            0,
            0.0001
    );

    private final TreeMap<Double, Double> estimatedExtendoAngles = new TreeMap<>();

    public static double tXAngle = 0;

    public static double secondsToExpire = 1;

    private LLResultTypes.ColorResult desiredSample;

    public boolean
            isSampleDetected = false,
            isOppositeSample = false,
            isYellowSample = false;


    public AutoAlignToSample(LimelightEx limelightEx) {
        this.limelightEx = limelightEx;

        headingPID.setGains(headingGains);

        // sets hashmap keys and values
        estimatedExtendoAngles.put(25.0, 96.75);
        estimatedExtendoAngles.put(37.5, 80.625);
        estimatedExtendoAngles.put(50.0, 64.5);
        estimatedExtendoAngles.put(62.5, 48.375);
        estimatedExtendoAngles.put(75.0, 32.25);
        estimatedExtendoAngles.put(87.5, 16.125);
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

        List<LLResultTypes.ColorResult> targets = limelightEx.getColorResult();

        // checks and returns detection
        if (targets != null || !targets.isEmpty()) {
            desiredSample = targets.get(0);

            return true;
        }

        return false;
    }

    private Double calculateExtendoTarget() {
        double currentArea = desiredSample.getTargetArea();

        if (estimatedExtendoAngles.containsKey(currentArea)) return estimatedExtendoAngles.get(currentArea);

        // x = tArea; y = target extendo angle

        double upperBound = estimatedExtendoAngles.ceilingKey(currentArea); // x2
        double lowerBound = estimatedExtendoAngles.floorKey(currentArea); // x1

        double upperValue = estimatedExtendoAngles.get(upperBound); // y2
        double lowerValue = estimatedExtendoAngles.get(lowerBound); // y1

        // linear interpolation formula
        return lowerValue + (currentArea - lowerBound) * ((upperValue - lowerValue) / (upperBound - lowerBound));
    }

    private PoseVelocity2d calculateHeadingTarget() {
        double theta = desiredSample.getTargetXDegrees();

        headingPID.setTarget(new State(tXAngle));

        State currentHeading = new State(theta);
        double headingPower = headingPID.calculate(currentHeading);

        return new PoseVelocity2d(
                new Vector2d(
                        0,
                        0
                ),
                headingPower
        );
    }

    private void setEdgeCases() {
        boolean ifBlueSample = robot.intake.getCurrentSample() == ColorRangefinderEx.SampleColor.BLUE;
        boolean ifRedSample = robot.intake.getCurrentSample() == ColorRangefinderEx.SampleColor.RED;
        boolean ifYellowSample = robot.intake.getCurrentSample() == ColorRangefinderEx.SampleColor.YELLOW;

        switch (targetColor) {
            case RED: {
                isOppositeSample = ifBlueSample;
                isYellowSample = ifYellowSample;
                break;
            }
            case BLUE: {
                isOppositeSample = ifRedSample;
                isYellowSample = ifYellowSample;
                break;
            }
            case YELLOW: {
                isOppositeSample = ifBlueSample || ifRedSample;
                break;
            }
        }
    }


    public Action driveToTarget() {
        return new Action() {
            boolean isFirstTime = true;
            final ElapsedTime expirationTimer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (isFirstTime) {
                    isFirstTime = false;
                    expirationTimer.startTime();
                }

                // update detections
                limelightEx.update();
                targetSample();

                // send out output to motors/servos
                robot.extendo.setTargetAngle(calculateExtendoTarget(), true);
                robot.drivetrain.setFieldCentricPowers(calculateHeadingTarget());

                robot.drivetrain.updatePoseEstimate();
//                setEdgeCases();

                return expirationTimer.seconds() <= secondsToExpire;
            }
        };
    }
}
