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
import com.acmerobotics.roadrunner.SequentialAction;
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
                intermediaryX = 25,
                intakeSampleY = -39,
                outtakeSampleY = -44,

                firstSampleX = 30.5,
                secondSampleX = 43,
                thirdSampleX = 45,

                giveFirstSampleX = 41,
                giveSecondSampleX = 44,
                giveThirdSampleX = 46,

                // Headings
                intakeFirstSampleHeading = 45,
                intakeSecondSampleHeading = 45,
                intakeThirdSampleHeading = 45,
                outtakeFirstSampleHeading = 330,
                outtakeSecondSampleHeading = 330,
                outtakeThirdSampleHeading = 330,
                firstExtendoAngle = 78,
                secondExtendoAngle = 78,
                thirdExtendoAngle = 78,

                // Timings
                sleepSecondsBeforeLimelightActivation = 0.5,
                sleepSecondsBeforeSubDetection = 0.8,
                firstSleepBeforeTurning = 0.1,
                secondSleepBeforeTurning = 0.1,
                thirdSleepBeforeTurning = 0.1,
                outtakeSampleDelay = 1,
                sleepBeforeV4B = 0.1,

                // Roller Power
                intakeRollerPower = 0.8,
                outtakeRollerPower = -0.8;

    }

    public static class ScoreSpecimens {
        public double
                // Positions
                subSampleX = 5,
                scoreSpecimenY = -33.5,

                wallPickupX = 41.5,
                intakeSpecimenY = -62.5,

                specimenOffsetX = -5.5,
                secondSpecimenOffsetY = 2.5,
                thirdSpecimenOffsetY = 2.5,
                fourthSpecimenOffsetY = 2.5,
                fifthSpecimenOffsetY = 2.5,
                sixthSpecimenOffsetY = 2.5,

                // Headings
                scoringAngle = 100,

                // Timings
                sleepBeforeGrab = 0,
                sleepSecondsBeforeUnclampFirst = 1.2,
                sleepSecondsBeforeUnclampSecond = 2,
                sleepSecondsBeforeUnclampThird = 2,
                sleepSecondsBeforeUnclampFourth = 2,
                sleepSecondsBeforeUnclampFifth = 2,
                sleepSecondsBeforeUnclampSixth = 2;
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
                .strafeToLinearHeading(new Vector2d(10 + offsetX, S_S.scoreSpecimenY + offsetY), Math.toRadians(S_S.scoringAngle))//, (pose2dDual, posePath, v) -> scoreSpecimenVelocityConstraint, new ProfileAccelConstraint(minScoreProfileAccel, maxScoreProfileAccel))
                .setTangent(Math.toRadians(315));

        if (!doPark) builder = builder.afterTime(0, RobotActions.setupWallPickup());

        builder = builder.splineToLinearHeading(new Pose2d(S_S.wallPickupX, S_S.intakeSpecimenY, Math.toRadians(90)), Math.toRadians(270));

        // Setting up for the next cycle
        if (!doPark) builder = builder.afterTime(S_S.sleepBeforeGrab, RobotActions.setupSpecimen());


        return builder;
    }

    private TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {

        builder = builder
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(S_S.wallPickupX, S_S.intakeSpecimenY, Math.toRadians(90)), Math.toRadians(270));

        builder = scoreSpecimen(builder, S_S.specimenOffsetX, S_S.secondSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampSecond);
        builder = scoreSpecimen(builder, S_S.specimenOffsetX, S_S.thirdSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampThird);
        builder = scoreSpecimen(builder, S_S.specimenOffsetX, S_S.fourthSpecimenOffsetY, false, S_S.sleepSecondsBeforeUnclampFourth);
        builder = scoreSpecimen(builder, S_S.specimenOffsetX, S_S.fifthSpecimenOffsetY, !isSixPlusZero, S_S.sleepSecondsBeforeUnclampFifth);

        if (isSixPlusZero) builder = scoreSpecimen(builder, S_S.specimenOffsetX, S_S.sixthSpecimenOffsetY, true, S_S.sleepSecondsBeforeUnclampSixth);

        return builder;
    }
    private TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .splineToSplineHeading(new Pose2d(G_S.intermediaryX, G_S.intakeSampleY, Math.toRadians(90)), Math.toRadians(0))
                // Intaking 1st
                .splineToSplineHeading(new Pose2d(G_S.firstSampleX, G_S.intakeSampleY, Math.toRadians(G_S.intakeFirstSampleHeading)), Math.toRadians(0))
                .stopAndAdd(new ParallelAction(
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                        RobotActions.setExtendo(G_S.firstExtendoAngle, 0),
                        RobotActions.setRollers(G_S.intakeRollerPower, G_S.firstSleepBeforeTurning)
                ))

                // Outtaking 1st
                .afterTime(G_S.outtakeSampleDelay, new SequentialAction(
                        RobotActions.setRollers(G_S.outtakeRollerPower, G_S.sleepBeforeV4B),
                        RobotActions.setV4B(Intake.V4BAngle.UP, 0)
                ))
                .strafeToLinearHeading(new Vector2d(G_S.giveFirstSampleX, G_S.outtakeSampleY), Math.toRadians(G_S.outtakeFirstSampleHeading))
                .afterTime(0, RobotActions.extendIntake(Extendo.Extension.ONE_FOURTH))

                // Intaking 2nd
                .strafeToLinearHeading(new Vector2d(G_S.secondSampleX, G_S.intakeSampleY), Math.toRadians(G_S.intakeSecondSampleHeading))
                .stopAndAdd(new ParallelAction(
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                        RobotActions.setExtendo(G_S.secondExtendoAngle, 0),
                        RobotActions.setRollers(G_S.intakeRollerPower, G_S.secondSleepBeforeTurning)
                ))
                // Outtaking 2nd
                .afterTime(G_S.outtakeSampleDelay, new SequentialAction(
                        RobotActions.setRollers(G_S.outtakeRollerPower, G_S.sleepBeforeV4B),
                        RobotActions.setV4B(Intake.V4BAngle.UP, 0)
                ))
                .strafeToLinearHeading(new Vector2d(G_S.giveSecondSampleX, G_S.outtakeSampleY), Math.toRadians(G_S.outtakeSecondSampleHeading))
                .afterTime(0, RobotActions.extendIntake(Extendo.Extension.ONE_FOURTH))

                //Intaking 3rd
                .strafeToLinearHeading(new Vector2d(G_S.thirdSampleX, G_S.intakeSampleY), Math.toRadians(G_S.intakeThirdSampleHeading))
                .stopAndAdd(new ParallelAction(
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                        RobotActions.setExtendo(G_S.thirdExtendoAngle, 0),
                        RobotActions.setRollers(G_S.intakeRollerPower, G_S.thirdSleepBeforeTurning)
                ))

                //Outtaking 3rd
                .afterTime(0, RobotActions.setExtendo(Extendo.Extension.ONE_HALF, 0))
                .afterTime(G_S.outtakeSampleDelay, new SequentialAction(
                        RobotActions.setRollers(G_S.outtakeRollerPower, G_S.sleepBeforeV4B),
                        RobotActions.setV4B(Intake.V4BAngle.UP, 0)
                ))
                .strafeToLinearHeading(new Vector2d(G_S.giveThirdSampleX, G_S.outtakeSampleY), Math.toRadians(G_S.outtakeThirdSampleHeading))
                .afterTime(0, new ParallelAction(
                        RobotActions.retractExtendo(),
                        RobotActions.setupWallPickup()
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
