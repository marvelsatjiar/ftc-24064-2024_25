package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.IS_RED;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.LIMELIGHT_BLUE_DETECTION_PIPELINE;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.LIMELIGHT_RED_DETECTION_PIPELINE;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public ColorRangefinderEx.SampleColor targetColor;

    private final PIDController axialPID = new PIDController();
    private final PIDController lateralPID = new PIDController();
    private final PIDController headingPID = new PIDController();

    public static PIDGains headingGains = new PIDGains(
            1.15,
            0,
            0.0001
    );

    public static PIDGains axialPIDGains = new PIDGains(
            0,
            0,
            0
    );

    public static PIDGains lateralPIDGains = new PIDGains(
            0.0288875,
            0.00007125,
            0.00001486525
    );

    public static double
        targetAxial = 1,
        targetLateral = 0;

    private LLResultTypes.ColorResult desiredSample;

    public ElapsedTime driveToTimer = new ElapsedTime();

    public int lockSampleCounter = 0;
    public boolean isSampleLocked = false;

    private boolean isExpired = false;

    public SubmersibleColorDetection(LimelightEx limelightEx) {
        this.limelightEx = limelightEx;

        axialPID.setGains(axialPIDGains);
        lateralPID.setGains(lateralPIDGains);
        headingPID.setGains(headingGains);

        axialPID.setTarget(new State(targetAxial));
        lateralPID.setTarget(new State(targetLateral));
        headingPID.setTarget(new State(Math.toRadians(90)));
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
        double theta = robot.drivetrain.headingOffset - robot.drivetrain.pose.heading.toDouble();

        if (theta < 0) theta += Math.PI * 2;

        State currentAxial = new State(desiredSample.getTargetArea());
        State currentLateral = new State(desiredSample.getTargetXDegrees());
        State currentHeading = new State(theta);
        
        double axialPower = axialPID.calculate(currentAxial);
        double headingPower = -headingPID.calculate(currentHeading);
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
                headingPower
        );
    }


    public Action driveToTarget() {
        return new Actions.SingleCheckAction(
                () -> robot.intake.getCurrentSample() != targetColor || isExpired,
                new SequentialAction(
                        new InstantAction(() -> driveToTimer.startTime()),
                        new InstantAction(() -> isExpired = driveToTimer.milliseconds() > 1000),
                        new InstantAction(() -> robot.drivetrain.setFieldCentricPowers(calculateTarget()))
                )
        );
    }
}
