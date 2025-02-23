package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.IS_RED;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.LIMELIGHT_BLUE_DETECTION_PIPELINE;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.LIMELIGHT_RED_DETECTION_PIPELINE;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.auto.Actions;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.sensor.ColorRangefinderEx;
import org.firstinspires.ftc.teamcode.sensor.vision.LimelightEx;

import java.util.List;

@Config
public class SubmersibleColorDetection {
    private final LimelightEx limelightEx;

    private ColorRangefinderEx.SampleColor targetColor;

    private final PIDController axialPID = new PIDController();
    private final PIDController lateralPID = new PIDController();

    public static PIDGains axialPIDGains = new PIDGains(
            0,
            0,
            0
    );

    public static PIDGains lateralPIDGains = new PIDGains(
            0,
            0,
            0
    );

    public static double
        targetAxial = 1,
        targetLateral = 1;

    private LLResultTypes.ColorResult desiredSample;

    public int lockSampleCounter = 0;
    public boolean isSampleLocked = false;

    public SubmersibleColorDetection(LimelightEx limelightEx) {
        this.limelightEx = limelightEx;

        axialPID.setGains(axialPIDGains);
        lateralPID.setGains(lateralPIDGains);

        axialPID.setTarget(new State(targetAxial));
        lateralPID.setTarget(new State(targetLateral));
    }

    public void activateLimelight() {
        targetColor = IS_RED ? ColorRangefinderEx.SampleColor.RED : ColorRangefinderEx.SampleColor.BLUE;

        limelightEx.enableStagelite(true);

        limelightEx.getLimelight().pipelineSwitch(IS_RED ? LIMELIGHT_RED_DETECTION_PIPELINE : LIMELIGHT_BLUE_DETECTION_PIPELINE);
        limelightEx.getLimelight().setPollRateHz(10);
    }

    // add lock mechanism here (for sample)- for this you must also get current detections!!!
    public boolean lockTargetSample() {
        limelightEx.update();

        List<LLResultTypes.ColorResult> targets = limelightEx.getColorResult();

        if (targets != null && !targets.isEmpty()) return false;

        desiredSample = targets.get(0);

        for (LLResultTypes.ColorResult target : targets) {
            if (target.getTargetArea() > desiredSample.getTargetArea()) desiredSample = target;
        }

        lockSampleCounter++;

        return true;
    }

    private PoseVelocity2d calculateTarget() {
        State currentAxial = new State(desiredSample.getTargetArea());
        State currentLateral = new State(desiredSample.getTargetXDegrees());
        
        double axialPower = axialPID.calculate(currentAxial);
        double lateralPower = lateralPID.calculate(currentLateral);

        double kMovement = 0.0573;
        if (axialPower < kMovement && axialPower > 0) axialPower = kMovement;
        else if (axialPower < 0 && axialPower > -kMovement) axialPower = kMovement;

        if (lateralPower < kMovement && lateralPower > 0) lateralPower = kMovement;
        else if (lateralPower < 0 && lateralPower > -kMovement) lateralPower = kMovement;

        return new PoseVelocity2d(
                new Vector2d(
                        axialPower,
                        lateralPower
                ),
                0
        );
    }


    public Action driveToTarget() {
        return new Actions.SingleCheckAction(
                () -> robot.intake.getCurrentSample() != targetColor,
                new InstantAction(() -> robot.drivetrain.setFieldCentricPowers(calculateTarget()))
        );
    }
}
