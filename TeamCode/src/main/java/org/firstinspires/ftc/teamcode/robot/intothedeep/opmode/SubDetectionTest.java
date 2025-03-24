package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.IS_RED;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.LIMELIGHT_BLUE_DETECTION_PIPELINE;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.LIMELIGHT_RED_DETECTION_PIPELINE;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.Actions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Claw;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement.AutoAlignToSample;

@Config
@Autonomous(name = "Sub Detection Test")
public class SubDetectionTest extends AbstractAuto{
    private AutoAlignToSample autoAlignToSample;

    public static double
            minSubExtendoAngle = 70,
            secondsToExpire = 4,
            startingPositionX = 7.375,
            startingPositionY = -62,
            scoreSpecimenY = -30.5,
            firstSpecimenOffsetY = 5,
            estimatedSixthSample = 10,
            sleepSecondsBeforeUnclampFirst = 1.2;

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(startingPositionX, startingPositionY, Math.toRadians(90));
    }

    @Override
    protected void configure() {
        super.configure();
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(A)) IS_RED = !IS_RED;

            if (gamepadEx1.wasJustPressed(DPAD_LEFT)) estimatedSixthSample--;
            if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) estimatedSixthSample++;

            mTelemetry.addLine("Estimated 6th sample is : " + estimatedSixthSample);
            mTelemetry.addLine("Alliance is : " + (IS_RED ? "RED" : "BLUE"));

            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }
    }

    @Override
    protected void onInit() {
        super.onInit();

        autoAlignToSample = new AutoAlignToSample(robot.limelightEx);

        robot.arm.setArmAngle(Arm.ArmAngle.NEUTRAL);
        robot.arm.setArmstendoAngle(Arm.Extension.RETRACTED);
        robot.arm.setWristAngle(Arm.WristAngle.COLLECTING);
        robot.claw.setAngle(Claw.ClawAngles.SPECIMEN_CLAMPED);
        robot.intake.setTargetV4BAngle(Intake.V4BAngle.UP);

        robot.intake.run(0);
        robot.arm.run();
        robot.claw.run();
    }

    @Override
    protected Action onRun() {
        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());

        builder = scoreFirstSpecimen(builder);

        return builder.build();
    }

    private TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
//                .afterTime(0, RobotActions.setupSpecimen())
                .afterTime(0, new SequentialAction(
                        new InstantAction(() -> autoAlignToSample.activateLimelight(IS_RED ? LIMELIGHT_RED_DETECTION_PIPELINE : LIMELIGHT_BLUE_DETECTION_PIPELINE)),
                        RobotActions.setExtendo(minSubExtendoAngle, 0)
                ))
//                .afterTime(sleepSecondsBeforeUnclampFirst, RobotActions.scoreSpecimen())
//                .splineToConstantHeading(new Vector2d(estimatedSixthSample, (scoreSpecimenY + firstSpecimenOffsetY)), Math.toRadians(90))
                .stopAndAdd(
                    new SequentialAction(
                        autoAlignToSample.detectTarget(secondsToExpire),
                        new InstantAction(() -> robot.limelightEx.enableStagelite(false))
                ))
                .stopAndAdd(new Actions.SingleCheckAction(
                        () -> autoAlignToSample.wasSampleDetected(),
                        new InstantAction(autoAlignToSample::driveToTarget)
                ));

        return builder;
    }

}
