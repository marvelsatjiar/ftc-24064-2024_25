package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
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
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Lift;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement.AutoAlignToSample;

@Autonomous(name = "Specimen 6+0")
@Config
public class Specimen6Plus0 extends AbstractAuto {
    private AutoAlignToSample autoAlignToSample;

    public boolean isSixPlusZero = false;

    public static double
            parkVelocityConstraint = 160,
            startingPositionX = 7.375,
            startingPositionY = -62,
            scoreSpecimenY = -30.5,
            dropoffFirstSampleX = 25,
            waitBeforeOuttakeSample = 0.4,
            waitToExtendTo2ndSample = 0.4,
            setBackWallPickupWait = 0.8,
            outtakeSampleDelay = 0.3,
            intakeSampleDelay = 0.5,
            getSampleX = 37.5,
            getThirdSampleX = 46,
            intakeSampleY = -32.5,
            outtakeSampleY = -44,
            outtakeSampleHeading = 300,
            intakeSampleHeading = 25,
            parkX = 23,
            parkY = -44.6,
            extendSleep = 0.2,
            secondSpecimenOffsetY = 2,
            thirdSpecimenOffsetY = 2.5,
            fourthSpecimenOffsetY = 2.5,
            fifthSpecimenOffsetY = 2.5,
            sixthSpecimenOffsetY = 2.5,
            secondSpecimenOffsetX = -11,
            thirdSpecimenOffsetX = -9,
            fourthSpecimenOffsetX = -5.5,
            fifthSpecimenOffsetX = -1,
            sixthSpecimenOffsetX = 0.5,
            sleepSecondsBeforeSetupSecond = 0.8,
            sleepSecondsBeforeSetupThird = 0.6,
            sleepSecondsBeforeSetupFourth = 0.7,
            sleepSecondsBeforeSetupFifth = 0.8,
            sleepSecondsBeforeSetupSixth = 0.9,
            bumpSpecimen = -62.5,
            bumpSecondSpecimen = -62,
            pickupSecondSpecimenX = 40,
            intakeSpecimenY = -56,
            wallPickupX = 41.5,
            startBumpToClampTime = 0.4,
            intakeSpecimenVelocityConstraint = 90,
            scoreSpecimenVelocityConstraint = 140,
            scoreFirstSpecimenVelocityConstraint = 140,
            maxProfileAccel = 60,
            minScoreProfileAccel = -50,
            maxScoreProfileAccel = 60,
            minFirstProfileAccel = -45,
            estimatedSixthSample = 5,
            scoreToRetractWait = 0.3,
            sleepSecondsBeforeLimelightActivation = 0.5,
            sleepSecondsBeforeSubDetection = 0.8,
            sleepSecondsBeforeUnclampFirst = 1.2,
            sleepSecondsBeforeUnclampSecond = 2.3,
            sleepSecondsBeforeUnclampThird = 2.1,
            sleepSecondsBeforeUnclampFourth = 2,
            sleepSecondsBeforeUnclampFifth = 2,
            sleepSecondsBeforeUnclampSixth = 1.9,
            secondSpecimenSleepBeforeSetup = 0.5,
            bumpSpecimenVelConstraint = 20;

    @Override
    protected void configure() {
        super.configure();
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
//         Get gamepad 1 button input and save "right" and "red" booleans for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (gamepadEx1.wasJustPressed(B)) Common.IS_RED = true;
            if (gamepadEx1.wasJustPressed(X)) Common.IS_RED = false;
            if (gamepadEx1.wasJustPressed(A)) isSixPlusZero = false;
            if (gamepadEx1.wasJustPressed(DPAD_LEFT)) estimatedSixthSample--;
            if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) estimatedSixthSample++;
            mTelemetry.addLine("| B - Red alliance | X - Blue alliance |");
            mTelemetry.addLine("| A - Toggle 6+0");
            mTelemetry.addLine("| DPAD LEFT - Subtract estimated 6th sample | DPAD RIGHT - Add estimated 6th sample");
            mTelemetry.addLine();
            mTelemetry.addLine("Selected alliance : " + (Common.IS_RED ? "Red" : "Blue"));
            mTelemetry.addLine("Estimated 6th sample is : " + estimatedSixthSample);
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

        robot.arm.setArmAngle(Arm.ArmAngle.SCORE_SPECIMEN);
        robot.arm.setWristAngle(Arm.WristAngle.SCORE_SPECIMEN);
        robot.arm.setArmstendoAngle(Arm.Extension.WALL_PICKUP);
        robot.lift.setTargetTicks(Lift.Ticks.SETUP_SPECIMEN);
        robot.claw.setAngle(Claw.ClawAngles.SAMPLE_CLAMPED);
        robot.setCurrentState(Robot.State.FRONT_WALL_PICKUP);

        robot.arm.run();
        robot.claw.run();
    }

    @Override
    protected Action onRun() {
        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());

        builder = scoreFirstSpecimen(builder);
        builder = giveSamples(builder);
        builder = scoreAllSpecimens(builder);
        builder = park(builder);

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
                .strafeToSplineHeading(new Vector2d(parkX, parkY), Math.toRadians(315), (pose2dDual, posePath, v) -> parkVelocityConstraint);
        return builder;
    }

    private TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder, double offsetX, double offsetY, boolean doPark, double sleepSecondsBeforeSetup, double sleepSecondsBeforeUnclamp) {
        // Scoring
        builder = builder
                .setTangent(90)
                .afterTime(sleepSecondsBeforeUnclamp, new SequentialAction(
                        new ParallelAction(
                                RobotActions.scoreSpecimen(),
                                new SleepAction(scoreToRetractWait)
                        ),
                        RobotActions.setArmstendo(Arm.Extension.RETRACTED, 0)
                ))

                .strafeToConstantHeading(new Vector2d(10 + offsetX, scoreSpecimenY + offsetY), (pose2dDual, posePath, v) -> scoreSpecimenVelocityConstraint, new ProfileAccelConstraint(minScoreProfileAccel, maxScoreProfileAccel))

                .setTangent(Math.toRadians(270))
                .afterTime(setBackWallPickupWait, RobotActions.setupWallPickup())
                .strafeToConstantHeading(new Vector2d(wallPickupX, intakeSpecimenY), (pose2dDual, posePath, v) -> scoreSpecimenVelocityConstraint, new ProfileAccelConstraint(minScoreProfileAccel, maxScoreProfileAccel));

        // Setting up for the next cycle
        if (!doPark) {
            builder = builder
                    .afterTime(startBumpToClampTime, RobotActions.setupSpecimenStable(sleepSecondsBeforeSetup))
                    .lineToY(bumpSpecimen, ((pose2dDual, posePath, v) -> bumpSpecimenVelConstraint));
            }

        return builder;
    }

    private TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {

        builder = builder
                .setTangent(180)
                .splineToSplineHeading(new Pose2d(pickupSecondSpecimenX, intakeSpecimenY, Math.toRadians(90)), Math.toRadians(270))
                .afterTime(startBumpToClampTime, RobotActions.setupSpecimenStable(secondSpecimenSleepBeforeSetup))
                .splineToSplineHeading(new Pose2d(pickupSecondSpecimenX, bumpSecondSpecimen, Math.toRadians(90)), Math.toRadians(270), (pose2dDual, posePath, v) -> bumpSpecimenVelConstraint);

        builder = scoreSpecimen(builder, secondSpecimenOffsetX, secondSpecimenOffsetY, false, sleepSecondsBeforeSetupSecond, sleepSecondsBeforeUnclampSecond);
        builder = scoreSpecimen(builder, thirdSpecimenOffsetX, thirdSpecimenOffsetY, false, sleepSecondsBeforeSetupThird, sleepSecondsBeforeUnclampThird);
        builder = scoreSpecimen(builder, fourthSpecimenOffsetX, fourthSpecimenOffsetY, false, sleepSecondsBeforeSetupFourth, sleepSecondsBeforeUnclampFourth);
        builder = scoreSpecimen(builder, fifthSpecimenOffsetX, fifthSpecimenOffsetY, !isSixPlusZero, sleepSecondsBeforeSetupFifth, sleepSecondsBeforeUnclampFifth);

        if (isSixPlusZero) builder = scoreSpecimen(builder, sixthSpecimenOffsetX, sixthSpecimenOffsetY, true, sleepSecondsBeforeSetupSixth, sleepSecondsBeforeUnclampSixth);

        return builder;
    }
    private TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(90))
                .afterTime(waitBeforeOuttakeSample, RobotActions.extendIntake(Extendo.Extension.ONE_HALF))
                .splineToConstantHeading(new Vector2d(dropoffFirstSampleX, outtakeSampleY), Math.toRadians(0))
                .afterTime(0, new SequentialAction(
                        RobotActions.setRollers(-1, outtakeSampleDelay),
                        RobotActions.setRollers(0, 0)
                ))

                .splineToSplineHeading(new Pose2d(getSampleX, intakeSampleY, Math.toRadians(intakeSampleHeading)), Math.toRadians(110))
                .afterTime(0, new SequentialAction(
                        RobotActions.setRollers(1, intakeSampleDelay),
                        RobotActions.setRollers(0, 0)
                ))
                .strafeToLinearHeading(new Vector2d(getSampleX, outtakeSampleY), Math.toRadians(outtakeSampleHeading), (pose2dDual, posePath, v) -> intakeSpecimenVelocityConstraint)
                .afterTime(0, new SequentialAction(
                        RobotActions.setRollers(-1, outtakeSampleDelay),
                        RobotActions.setRollers(0, 0)
                ))

                .afterTime(waitToExtendTo2ndSample, RobotActions.extendIntake(Extendo.Extension.THREE_FOURTHS))
                .strafeToLinearHeading(new Vector2d(getSampleX, intakeSampleY), Math.toRadians(getSampleX), (pose2dDual, posePath, v) -> intakeSpecimenVelocityConstraint)
                .afterTime(0, new SequentialAction(
                        RobotActions.setRollers(1, intakeSampleDelay),
                        RobotActions.setRollers(0, 0)
                ))
                .strafeToLinearHeading(new Vector2d(getSampleX, outtakeSampleY), Math.toRadians(outtakeSampleHeading), (pose2dDual, posePath, v) -> intakeSpecimenVelocityConstraint)
                .afterTime(0, new SequentialAction(
                        RobotActions.setRollers(-1, outtakeSampleDelay),
                        RobotActions.setRollers(0, 0)
                ))

                .setTangent(90)
                .splineToSplineHeading(new Pose2d(getThirdSampleX, intakeSampleY, Math.toRadians(intakeSampleHeading)), Math.toRadians(intakeSampleHeading))
                .afterTime(0, new SequentialAction(
                        RobotActions.setRollers(1, intakeSampleDelay),
                        RobotActions.setRollers(0, 0)
                ))
                .strafeToLinearHeading(new Vector2d(getThirdSampleX, outtakeSampleY), Math.toRadians(outtakeSampleHeading), (pose2dDual, posePath, v) -> intakeSpecimenVelocityConstraint)
                .afterTime(0, new SequentialAction(
                        RobotActions.setRollers(-1, outtakeSampleDelay),
                        RobotActions.setRollers(0, 0),
                        RobotActions.retractExtendo()
                ));

        return builder;
    }

    private TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(0, RobotActions.setupSpecimen())
                .afterTime(sleepSecondsBeforeLimelightActivation, new ParallelAction(
                        new InstantAction(() -> autoAlignToSample.activateLimelight()),
                        RobotActions.extendIntake(Extendo.Extension.THREE_FOURTHS)
                ))
                .afterTime(sleepSecondsBeforeSubDetection, new Actions.SingleCheckAction(
                        () -> autoAlignToSample.lockSampleCounter != 9,
                        new InstantAction(() -> autoAlignToSample.isSampleLocked = autoAlignToSample.lockTargetSample())
                ))
                .afterTime(sleepSecondsBeforeUnclampFirst, new SequentialAction(
                        RobotActions.scoreSpecimen(),
                        new SleepAction(scoreToRetractWait),
                        RobotActions.retractToNeutral(0)
                ))
                .splineToConstantHeading(new Vector2d(estimatedSixthSample, scoreSpecimenY), Math.toRadians(90), (pose2dDual, posePath, v) -> scoreFirstSpecimenVelocityConstraint, new ProfileAccelConstraint(minFirstProfileAccel, maxProfileAccel));

        if (autoAlignToSample.isSampleLocked) {
            builder = builder
                    .stopAndAdd(new SequentialAction(
                            new ParallelAction(
                                    RobotActions.setRollers(1, 0),
                                    autoAlignToSample.driveToTarget()
                            ),
                            RobotActions.retractExtendo()
                    ));
        }

        return builder;
    }
}
