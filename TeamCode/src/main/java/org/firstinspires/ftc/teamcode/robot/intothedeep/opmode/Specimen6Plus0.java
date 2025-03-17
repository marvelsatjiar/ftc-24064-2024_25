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
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Claw;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.enhancement.AutoAlignToSample;

@Autonomous(name = "Specimen 6+0")
@Config
public class Specimen6Plus0 extends AbstractAuto {
    private AutoAlignToSample autoAlignToSample;

    public boolean isSixPlusZero = false;

    public static class GiveSamples {
        public double
                // Position
                intermediaryX = 22,
                intermediaryY = -39,

                intakeSampleX = 52,
                intakeSampleY = -41.5,

                outtakeSampleX = 52,
                outtakeSampleY = -41.5,


                // Headings
                intakeFirstSampleHeading = 91,
                intakeSecondSampleHeading = 58.5,
                intakeThirdSampleHeading = 38,
                outtakeFirstSampleHeading = -70,
                outtakeSecondSampleHeading = -70,
                outtakeThirdSampleHeading = -70,
                intakeFirstExtendoAngle = 80,
                intakeSecondExtendoAngle = 71.5,
                intakeThirdExtendoAngle = 130,
                outtakeExtendoAngleFirst = 50,
                outtakeExtendoAngleSecond = 50,
                outtakeExtendoAngleThird = 50,

                // Timings
                sleepSecondsBeforeLimelightActivation = 0.5,
                sleepSecondsBeforeSubDetection = 0.8,
                firstIntakeDelay = 0.5,
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
                subSampleX = 5,
                scoreSpecimenY = -33.5,

                wallPickupX = 33.5,
                secondWallPickupX = 52,
                intakeSpecimenY = -56.5,
                intakeSecondSpecimenY = -47,
                intakeSecondBumpSpecimenY = -64,
                specimenOffsetX = -16,
                secondSpecimenOffsetY = 13,
                thirdSpecimenOffsetY = 13,
                fourthSpecimenOffsetY = 13,
                fifthSpecimenOffsetY = 13,
                sixthSpecimenOffsetY = 13,

                // Headings
                scoringAngle = 105,

                // Timings
                timeBeforeWallPickup = 1,
                timeBeforeWrist = 0.2,
                timeBeforeMoving = 0.1,
                lastFourSleepBeforeGrab = 0,
                secondSleepBeforeSetup = 0.1,
                sleepSecondsBeforeUnclampFirst = 1.3,
                sleepSecondsBeforeUnclampSecond = 1.9,
                sleepSecondsBeforeUnclampThird = 1.5,
                sleepSecondsBeforeUnclampFourth = 1.5,
                sleepSecondsBeforeUnclampFifth = 1.5,
                sleepSecondsBeforeUnclampSixth = 2,


                //Constraints
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
            if (gamepadEx1.wasJustPressed(A)) isSixPlusZero = !isSixPlusZero;
            if (gamepadEx1.wasJustPressed(DPAD_LEFT)) S_S.subSampleX--;
            if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) S_S.subSampleX++;
            mTelemetry.addLine("| B - Red alliance | X - Blue alliance |");
            mTelemetry.addLine("| A - Toggle 6+0");
            mTelemetry.addLine("| DPAD LEFT - Subtract estimated 6th sample | DPAD RIGHT - Add estimated 6th sample");
            mTelemetry.addLine();
            mTelemetry.addLine("Selected alliance : " + (Common.IS_RED ? "Red" : "Blue"));
            mTelemetry.addData("Six Plus Zero : ", isSixPlusZero);
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
                .setTangent(90)
                .afterTime(sleepSecondsBeforeUnclamp, RobotActions.scoreSpecimen())
                .strafeToLinearHeading(new Vector2d(10 + offsetX, S_S.scoreSpecimenY + offsetY), Math.toRadians(S_S.scoringAngle), (pose2dDual, posePath, v) -> S_S.scoreSpecimenVelocityConstraint, new ProfileAccelConstraint(S_S.minScoreProfileAccel, S_S.maxScoreProfileAccel))
                .setTangent(Math.toRadians(315));

        if (!doPark) builder = builder.afterTime(S_S.timeBeforeWallPickup, RobotActions.setupWallPickup());

        builder = builder.splineToLinearHeading(new Pose2d(S_S.wallPickupX, S_S.intakeSpecimenY, Math.toRadians(90)), Math.toRadians(S_S.scoringAngle), (pose2dDual, posePath, v) -> S_S.wallPickUpVelocityConstraint);

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
                .strafeToLinearHeading(new Vector2d(S_S.secondWallPickupX, S_S.intakeSecondSpecimenY), Math.toRadians(90))
                .lineToY(S_S.intakeSecondBumpSpecimenY)
                .stopAndAdd(new SequentialAction(
                        RobotActions.setClaw(Claw.ClawAngles.SPECIMEN_CLAMPED, S_S.timeBeforeWrist),
                        RobotActions.setWrist(Arm.WristAngle.GRAB_OFF_WALL, S_S.timeBeforeMoving)
                ))
                .afterTime(S_S.secondSleepBeforeSetup, RobotActions.setupSpecimen());

        builder = scoreSpecimen(builder, S_S.specimenOffsetX, S_S.secondSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampSecond);
        builder = scoreSpecimen(builder, S_S.specimenOffsetX, S_S.thirdSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampThird);
        builder = scoreSpecimen(builder, S_S.specimenOffsetX, S_S.fourthSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampFourth);
        builder = scoreSpecimen(builder, S_S.specimenOffsetX, S_S.fifthSpecimenOffsetY, !isSixPlusZero, S_S.sleepSecondsBeforeUnclampFifth);

        if (isSixPlusZero) builder = scoreSpecimen(builder, S_S.specimenOffsetX, S_S.sixthSpecimenOffsetY, true, S_S.sleepSecondsBeforeUnclampSixth);

        return builder;
    }
    private TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .splineToSplineHeading(new Pose2d(G_S.intermediaryX, G_S.intermediaryY, Math.toRadians(90)), Math.toRadians(0))
                // Intaking 1st
                .afterTime(G_S.firstIntakeDelay, new ParallelAction(
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                        RobotActions.setExtendo(G_S.intakeFirstExtendoAngle, 0),
                        RobotActions.setRollers(G_S.intakeRollerPower, 0)
                ))
                .splineToSplineHeading(new Pose2d(G_S.intakeSampleX, G_S.intakeSampleY, Math.toRadians(G_S.intakeFirstSampleHeading)), Math.toRadians(0))
                .waitSeconds(G_S.firstSleepBeforeTurning)
                // Outtaking 1st
                .afterTime(0, RobotActions.setExtendo(G_S.outtakeExtendoAngleFirst, 0))
                .afterTime(G_S.outtakeFirstSampleDelay, new SequentialAction(
                        RobotActions.setRollers(G_S.outtakeRollerPower, G_S.sleepBeforeV4B),
                        RobotActions.setV4B(Intake.V4BAngle.UP, 0)
                ))
                .strafeToLinearHeading(new Vector2d(G_S.outtakeSampleX, G_S.outtakeSampleY), Math.toRadians(G_S.outtakeFirstSampleHeading))
//                 Intaking 2nd
                .afterTime(G_S.secondIntakeDelay, new ParallelAction(
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                        RobotActions.setExtendo(G_S.intakeSecondExtendoAngle, 0),
                        RobotActions.setRollers(G_S.intakeRollerPower, 0)
                ))
                .strafeToLinearHeading(new Vector2d(G_S.intakeSampleX, G_S.intakeSampleY), Math.toRadians(G_S.intakeSecondSampleHeading))
                .waitSeconds(G_S.secondSleepBeforeTurning)
                // Outtaking 2nd
                .afterTime(0, RobotActions.setExtendo(G_S.outtakeExtendoAngleSecond, 0))
                .afterTime(G_S.outtakeSecondSampleDelay, new SequentialAction(
                        RobotActions.setRollers(G_S.outtakeRollerPower, G_S.sleepBeforeV4B),
                        RobotActions.setV4B(Intake.V4BAngle.UP, 0)
                ))

                .strafeToLinearHeading(new Vector2d(G_S.outtakeSampleX, G_S.outtakeSampleY), Math.toRadians(G_S.outtakeSecondSampleHeading))
//
                //Intaking 3rd
                .afterTime(G_S.thirdIntakeDelay, new ParallelAction(
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                        RobotActions.setExtendo(G_S.intakeThirdExtendoAngle, 0),
                        RobotActions.setRollers(G_S.intakeRollerPower, 0)
                ))
                .strafeToLinearHeading(new Vector2d(G_S.intakeSampleX, G_S.intakeSampleY), Math.toRadians(G_S.intakeThirdSampleHeading))
                .waitSeconds(G_S.thirdSleepBeforeTurning)
                //Outtaking 3rd
                .afterTime(0, RobotActions.setExtendo(G_S.outtakeExtendoAngleThird, 0))
                .afterTime(G_S.outtakeThirdSampleDelay, new SequentialAction(
                        RobotActions.setRollers(G_S.outtakeRollerPower, G_S.thirdSleepBeforeV4B),
                        RobotActions.setV4B(Intake.V4BAngle.UP, 0)
                ))
                .strafeToLinearHeading(new Vector2d(G_S.outtakeSampleX, G_S.outtakeSampleY), Math.toRadians(G_S.outtakeThirdSampleHeading))
                .afterTime(0, new ParallelAction(
                        RobotActions.setExtendo(Extendo.Extension.RETRACTED, 0),
                        RobotActions.setupWallPickup(),
                        new SleepAction(G_S.stopRollerDelay),
                        RobotActions.setRollers(0,0)
                ));

        return builder;
    }

    private TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
//        if (isSixPlusZero) {
//            builder = builder
//                    .afterTime(sleepSecondsBeforeLimelightActivation, new ParallelAction(
//                            new InstantAction(() -> autoAlignToSample.activateLimelight()),
//                            RobotActions.extendIntake(Extendo.Extension.THREE_FOURTHS)
//                    ))
//                    .afterTime(sleepSecondsBeforeSubDetection, new Actions.SingleCheckAction(
//                            () -> autoAlignToSample.lockSampleCounter != 9,
//                            new InstantAction(() -> autoAlignToSample.isSampleLocked = autoAlignToSample.lockTargetSample())
//                    ));
//        }

        builder = builder
                .afterTime(0, RobotActions.setupSpecimen())
                .afterTime(S_S.sleepSecondsBeforeUnclampFirst, RobotActions.scoreSpecimen())
                .strafeToConstantHeading(new Vector2d(S_S.subSampleX, S_S.scoreSpecimenY));//, (pose2dDual, posePath, v) -> scoreFirstSpecimenVelocityConstraint, new ProfileAccelConstraint(minFirstProfileAccel, maxProfileAccel));

//        if (autoAlignToSample.isSampleLocked && isSixPlusZero) {
//            builder = builder
//                    .stopAndAdd(new SequentialAction(
//                            new ParallelAction(
//                                    RobotActions.setRollers(1, 0),
//                                    autoAlignToSample.driveToTarget()
//                            ),
//                            RobotActions.retractExtendo()
//                    ));
//        }

        return builder;
    }
}
