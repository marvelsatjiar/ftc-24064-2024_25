package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.IS_RED;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.LIMELIGHT_BLUE_DETECTION_PIPELINE;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.LIMELIGHT_RED_DETECTION_PIPELINE;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.Actions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Claw;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement.AutoAlignToSample;
import org.firstinspires.ftc.teamcode.sensor.ColorRangefinderEx;

@Autonomous(name = "Specimen 6+0")
@Config
public class Specimen6Plus0 extends AbstractAuto {
    private AutoAlignToSample autoAlignToSample;

    public boolean isSevenPlusZero = true;

    public static class GiveSamples {
        public double
                // Position
                intermediaryX = 42,
                intermediaryY = -47,

                secondIntermediaryX = 35,
                secondIntermediaryY = -30,

                intakeSampleX = 52,
                intakeSampleY = -41.5,

                outtakeSampleX = 52,
                outtakeSampleY = -41.5,
                sample1X = 50,
                sample1Y = -5,
                sample2X = 56,
                sample2Y = -14,
                sample3X = 63,
                sample3Y = -14,
                giveSampleY = -52,

                //Headings
                goingToSampleTangent = 135,
                intermediaryTangent = 165,

                // Constraints
                giveSampleVelocityConstraint = 50,
                giveSampleMinAccelConstraint = -60,
                giveSampleMaxAccelConstraint = 60,
                goingToSampleVelocityConstraint = 50,
                goingToSampleMaxAccelConstraint = 60,
                goingToSampleMinAccelConstraint = -60,


                // Timings
                waitBeforeMoving = 1,
                sleepSecondsBeforeDrop = 0.2,
                sleepBeforeInterleaveSample = 1,
                delayBeforeV4B = 0.1,
                firstIntakeDelay = 0.7,
                secondIntakeDelay = 0.7,
                thirdIntakeDelay = 0.5,
                firstSleepBeforeTurning = 0,
                secondSleepBeforeTurning = 0.5,
                thirdSleepBeforeTurning = 0.5,
                outtakeFirstSampleDelay = 0.7,
                outtakeSecondSampleDelay = 0.5,
                outtakeThirdSampleDelay = 0.4,
                sleepBeforeV4B = 0.3,
                thirdSleepBeforeV4B = 1,
                stopRollerDelay = 0.7,

                // Roller Power
                intakeRollerPower = 1,
                outtakeRollerPower = -1;
    }
    public static class ScoreSpecimens {
        public double
                // Positions
                subSampleX = 0,
                scoreSpecimenY = -23.5,
                waitBefore6thSampleIntake = 0.7,
                wallPickupX = 33.5,
                secondWallPickupX = 52,
                intakeSpecimenY = -65.5,
                intakeSixthSpecimenY = -61.5,
                intakeSecondSpecimenY = -47,
                intakeSecondBumpSpecimenY = -63.5,
                specimen2ndOffsetX = -19,
                specimen3rdOffsetX = -20,
                specimen4thOffsetX = -18,
                specimen5thOffsetX = -16,
                specimen6thOffsetX = -15,
                specimen7thOffsetX = -13,
                secondSpecimenOffsetY = 4,
                thirdSpecimenOffsetY = 4,
                fourthSpecimenOffsetY = 4,
                fifthSpecimenOffsetY = 4,
                sixthSpecimenOffsetY = 4,
                seventhSpecimenOffsetY = 4,

                // Headings
                scoringAngle = 105,
                wallPickupTangent = 270,

                // Timings
                waitBeforeDetection = 0.15,
                waitUntilExtendoRetracted = 0.4,
                retractAfterIntakeSub = 0,
                secondsToExpire = 1.5,
                timeBeforeWallPickup = 1,
                timeBeforeWrist = 0.2,
                timeBeforeMoving = 0.1,
                lastFourSleepBeforeGrab = 0,
                secondSleepBeforeSetup = 0.1,
                sleepSecondsBeforeUnclampFirst = 1.4,
                sleepSecondsBeforeUnclampSecond = 2,
                sleepSecondsBeforeUnclampThird = 1.5,
                sleepSecondsBeforeUnclampFourth = 1.5,
                sleepSecondsBeforeUnclampFifth = 1.5,
                sleepSecondsBeforeUnclampSixth = 1.3,
                sleepSecondsBeforeUnclampSeventh = 1.5,
                timeBeforeInterleave = 0.3,
                v4bWait = 0.5,
                sleepBeforeTransfer = 1,


                //Constraints
                scoreFirstSpecimenVelocityConstraint = 160,
                maxFirstProfileAccel = -150,
                minFirstProfileAccel = 150,
                wallPickUpVelocityConstraint = 60,
                scoreSpecimenVelocityConstraint = 160,
                minScoreProfileAccel = -140,
                maxScoreProfileAccel = 150;
    }
    public static GiveSamples G_S = new GiveSamples();
    public static ScoreSpecimens S_S = new ScoreSpecimens();


    public static double
            startingPositionX = 7.375,
            startingPositionY = -60,
            parkX = 23,
            parkY = -44.6,
            extendSleep = 0.2;

    @Override
    protected void configure() {
        super.configure();
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
//         Get gamepad 1 button input and save "right" and "red" booleans for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (gamepadEx1.wasJustPressed(B)) Common.IS_RED = true;
            if (gamepadEx1.wasJustPressed(X)) Common.IS_RED = false;
            if (gamepadEx1.wasJustPressed(A)) isSevenPlusZero = !isSevenPlusZero;
            if (gamepadEx1.wasJustPressed(DPAD_LEFT)) S_S.subSampleX--;
            if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) S_S.subSampleX++;
            mTelemetry.addLine("| B - Red alliance | X - Blue alliance |");
            mTelemetry.addLine("| A - Toggle 6+0");
            mTelemetry.addLine("| DPAD LEFT - Subtract estimated 6th sample | DPAD RIGHT - Add estimated 6th sample");
            mTelemetry.addLine();
            mTelemetry.addLine("Selected alliance : " + (Common.IS_RED ? "Red" : "Blue"));
            mTelemetry.addData("Six Plus Zero : ", isSevenPlusZero);
            mTelemetry.addLine("Estimated 6th sample is : " + S_S.subSampleX);
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }
    }

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(startingPositionX, startingPositionY, Math.toRadians(90));
    }

    @Override
    protected void onInit() {
        super.onInit();

        autoAlignToSample = new AutoAlignToSample(robot.limelightEx);

        robot.arm.setArmAngle(Arm.ArmAngle.WALL_PICKUP);
        robot.arm.setWristAngle(Arm.WristAngle.GRAB_OFF_WALL);
        robot.arm.setArmstendoAngle(Arm.Extension.RETRACTED);
        robot.claw.setAngle(Claw.ClawAngles.SPECIMEN_CLAMPED);
        robot.setCurrentState(Robot.State.WALL_PICKUP);

        robot.arm.run();
        robot.claw.run();
    }

    @Override
    protected Action onRun() {
        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());

        builder = scoreFirstSpecimen(builder);
        builder = giveSamples(builder);
        builder = scoreAllSpecimens(builder);
//        builder = park(builder);

        return builder.build();
    }

    private TrajectoryActionBuilder park(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(extendSleep, new ParallelAction(
                        RobotActions.setExtendo(Extendo.Extension.EXTENDED,0),
                        RobotActions.setArm(Arm.ArmAngle.BASKET,0),
                        RobotActions.setWrist(Arm.WristAngle.BASKET,0),
                        RobotActions.setV4B(Intake.V4BAngle.UP, 0)
                ))
                .strafeToSplineHeading(new Vector2d(parkX, parkY), Math.toRadians(315));
        return builder;
    }

    private TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder, double offsetX, double offsetY, boolean doPark, double sleepSecondsBeforeUnclamp) {
        // Scoring
        builder = builder
                .afterTime(sleepSecondsBeforeUnclamp, RobotActions.scoreSpecimen());

        builder = builder.strafeToLinearHeading(new Vector2d(10 + offsetX, S_S.scoreSpecimenY + offsetY), Math.toRadians(S_S.scoringAngle), (pose2dDual, posePath, v) -> S_S.scoreSpecimenVelocityConstraint, new ProfileAccelConstraint(S_S.minScoreProfileAccel, S_S.maxScoreProfileAccel));

        if (!doPark) builder = builder.afterTime(S_S.timeBeforeWallPickup, RobotActions.setupWallPickup());

        builder = builder
                .setTangent(Math.toRadians(S_S.wallPickupTangent))
                .splineToLinearHeading(new Pose2d(S_S.wallPickupX, S_S.intakeSpecimenY, Math.toRadians(90)), Math.toRadians(270), (pose2dDual, posePath, v) -> S_S.wallPickUpVelocityConstraint);

        // Setting up for the next cycle
        if (!doPark) builder = builder
                .stopAndAdd(new SequentialAction(
                        RobotActions.setClaw(Claw.ClawAngles.SPECIMEN_CLAMPED, S_S.timeBeforeWrist),
                        RobotActions.setWrist(Arm.WristAngle.GRAB_OFF_WALL, S_S.timeBeforeMoving)
                ))
                .afterTime(S_S.lastFourSleepBeforeGrab, RobotActions.setupSpecimen());

        return builder;
    }

    private TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {

        builder = builder
                .setTangent(Math.toRadians(270))
                .lineToY(S_S.intakeSecondBumpSpecimenY)
                .stopAndAdd(new SequentialAction(
                        RobotActions.setClaw(Claw.ClawAngles.SPECIMEN_CLAMPED, S_S.timeBeforeWrist),
                        RobotActions.setWrist(Arm.WristAngle.GRAB_OFF_WALL, S_S.timeBeforeMoving)
                ))
                .afterTime(S_S.secondSleepBeforeSetup, RobotActions.setupSpecimen());

        builder = scoreSpecimen(builder, S_S.specimen2ndOffsetX, S_S.secondSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampSecond);
        builder = scoreSpecimen(builder, S_S.specimen3rdOffsetX, S_S.thirdSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampThird);
        builder = scoreSpecimen(builder, S_S.specimen4thOffsetX, S_S.fourthSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampFourth);
        builder = scoreSpecimen(builder, S_S.specimen5thOffsetX, S_S.fifthSpecimenOffsetY, !isSevenPlusZero, S_S.sleepSecondsBeforeUnclampFifth);
        if (isSevenPlusZero) builder = scoreSpecimen(builder, S_S.specimen6thOffsetX, S_S.sixthSpecimenOffsetY, true, S_S.sleepSecondsBeforeUnclampSixth);

        return builder;
    }

    private TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(270))
                .afterTime(0, RobotActions.transfer())
                .waitSeconds(G_S.waitBeforeMoving)
                .afterTime(G_S.sleepBeforeInterleaveSample, RobotActions.interleaveDropSample(G_S.sleepSecondsBeforeDrop))
                .splineToConstantHeading(new Vector2d(G_S.intermediaryX,G_S.intermediaryY), Math.toRadians(G_S.intermediaryTangent)) //,(pose2dDual, posePath, v) -> G_S.intermediaryVelocityConstraint, new ProfileAccelConstraint(G_S.intermediarySampleMinAccelConstraint, G_S.intermediarySampleMaxAccelConstraint))
                .splineToConstantHeading(new Vector2d(G_S.secondIntermediaryX, G_S.secondIntermediaryY), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(G_S.sample1X, G_S.sample1Y), Math.toRadians(270), (pose2dDual, posePath, v) -> G_S.goingToSampleVelocityConstraint, new ProfileAccelConstraint(G_S.goingToSampleMinAccelConstraint, G_S.goingToSampleMaxAccelConstraint))
                .lineToY(G_S.giveSampleY, (pose2dDual, posePath, v) -> G_S.giveSampleVelocityConstraint, new ProfileAccelConstraint(G_S.giveSampleMinAccelConstraint, G_S.giveSampleMaxAccelConstraint))
                .afterTime(0, RobotActions.setupWallPickup())
                .setTangent(Math.toRadians(G_S.goingToSampleTangent))
                .splineToLinearHeading(new Pose2d(G_S.sample2X, G_S.sample2Y, Math.toRadians(90)), Math.toRadians(270),(pose2dDual, posePath, v) -> G_S.goingToSampleVelocityConstraint, new ProfileAccelConstraint(G_S.goingToSampleMinAccelConstraint, G_S.goingToSampleMaxAccelConstraint))
                .setTangent(Math.toRadians(270))
                .lineToY(G_S.giveSampleY, (pose2dDual, posePath, v) -> G_S.giveSampleVelocityConstraint, new ProfileAccelConstraint(G_S.giveSampleMinAccelConstraint, G_S.giveSampleMaxAccelConstraint))
                .setTangent(Math.toRadians(G_S.goingToSampleTangent))
                .splineToLinearHeading(new Pose2d(G_S.sample3X, G_S.sample3Y, Math.toRadians(90)), Math.toRadians(270), (pose2dDual, posePath, v) -> G_S.goingToSampleVelocityConstraint, new ProfileAccelConstraint(G_S.goingToSampleMinAccelConstraint, G_S.goingToSampleMaxAccelConstraint))
                .setTangent(Math.toRadians(270))
                .lineToY(G_S.giveSampleY, (pose2dDual, posePath, v) -> G_S.giveSampleVelocityConstraint, new ProfileAccelConstraint(G_S.giveSampleMinAccelConstraint, G_S.giveSampleMaxAccelConstraint));

        return builder;
    }

    private TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(0, RobotActions.setupSpecimen())
                .afterTime(0, new InstantAction(() -> autoAlignToSample.activateLimelight(IS_RED ? LIMELIGHT_RED_DETECTION_PIPELINE : LIMELIGHT_BLUE_DETECTION_PIPELINE, IS_RED ? ColorRangefinderEx.SampleColor.RED : ColorRangefinderEx.SampleColor.BLUE)))
                .afterTime(S_S.sleepSecondsBeforeUnclampFirst, RobotActions.scoreSpecimen())
                .strafeToConstantHeading(new Vector2d(S_S.subSampleX, S_S.scoreSpecimenY), (pose2dDual, posePath, v) -> S_S.scoreFirstSpecimenVelocityConstraint);

        if (isSevenPlusZero) {
            builder = builder
                    .waitSeconds(S_S.waitBeforeDetection)
                    .stopAndAdd(
                            new SequentialAction(
                                    autoAlignToSample.detectTarget(S_S.secondsToExpire, true),
                                    new InstantAction(() -> robot.limelightEx.enableStagelite(false))
                            ))
                    .stopAndAdd(new SequentialAction(
                            new InstantAction(autoAlignToSample::generateTargetTrajectory),
                            telemetryPacket -> {
                                robot.run();
                                return autoAlignToSample.getTargetSampleTrajectory().run(telemetryPacket);
                            }
                    ));
        }

        return builder;
    }

    private Action scoreSixthAndSeventhSpecimen(TrajectoryActionBuilder builder) {
        builder
                .setTangent(Math.toRadians(315))
                .afterTime(G_S.sleepBeforeInterleaveSample, new SequentialAction(
                        RobotActions.transfer(),
                        RobotActions.interleaveDropSample(0))
                )
                .waitSeconds(S_S.waitBefore6thSampleIntake)
                .splineToLinearHeading(new Pose2d(S_S.wallPickupX, S_S.intakeSixthSpecimenY, Math.toRadians(90)), Math.toRadians(270), (pose2dDual, posePath, v) -> S_S.wallPickUpVelocityConstraint)
                .lineToY(S_S.intakeSpecimenY)
                .stopAndAdd(new SequentialAction(
                        RobotActions.setClaw(Claw.ClawAngles.SPECIMEN_CLAMPED, S_S.timeBeforeWrist),
                        RobotActions.setWrist(Arm.WristAngle.GRAB_OFF_WALL, S_S.timeBeforeMoving)
                ))

                .afterTime(0, RobotActions.setupSpecimen())
                .afterTime(S_S.sleepSecondsBeforeUnclampSixth, RobotActions.scoreSpecimen())
                .strafeToLinearHeading(new Vector2d(10 + S_S.specimen6thOffsetX, S_S.scoreSpecimenY + S_S.sixthSpecimenOffsetY), Math.toRadians(90), (pose2dDual, posePath, v) -> S_S.scoreSpecimenVelocityConstraint, new ProfileAccelConstraint(S_S.minScoreProfileAccel, S_S.maxScoreProfileAccel));

        return builder.build();
    }
}
