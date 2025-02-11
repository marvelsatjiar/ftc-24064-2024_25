package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
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
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Sweeper;

@Autonomous(name = "Specimen 5+0")
@Config
public class Specimen5Plus0 extends AbstractAuto {
    private boolean
            is5plus0 = true,
            usePartnerSpec = false;

    public static double
            parkVelocityConstraint = 160,
            startingPositionX = 7.375,
            startingPositionY = -62,
            scoreSpecimenY = -33.5,
            parkX = 23,
            parkY = -44.6,
            extendSleep = 0.2,
            secondSpecimenOffsetY = 1,
            thirdSpecimenOffsetY = 1,
            fourthSpecimenOffsetY = 1,
            fifthSpecimenOffsetY = 1,
            secondSpecimenOffsetX = -11,
            thirdSpecimenOffsetX = -9,
            fourthSpecimenOffsetX = -5.5,
            fifthSpecimenOffsetX = -1,
            sample1X = 47,
            sample2X = 54.5,
            sample3X = 63,
            startFirstSampleY = -12,
            startSampleY = -14,
            giveSample2Y = -44.5,
            bumpSpecimen = -62.5,
            bumpSecondSpecimen = -62,
            intakeSpecimenY = -56,
            giveSample1X = sample1X - 4,
            giveSample2X = sample2X - 3,
            giveSample3X = sample3X,
            giveSampleY = -45.5,
            giveSample3Y = -45,
            wallPickupX = 41.5,
            firstWallPickupX = 58,
            secondSweeperSleep = 0.7,
            thirdSweeperSleep = 0.4,
            startBumpToClampTime = 0.4,
            secondSpecimenStartBumpToClampTime = 0.2,
            givingSampleAngle = 270,
            setupFrontWallPickupWait = 0.2,
            scoreSpecimenVelocityConstraint = 140,
            giveSampleVelocityConstraint = 27,
            scoreFirstSpecimenVelocityConstraint = 140,
            giveSecondSampleSweeperWait = 0.7,
            giveFirstSampleSweeperWait = 0.4,
            minProfileAccel = -30,
            maxProfileAccel = 60,
            minScoreProfileAccel = -50,
            maxScoreProfileAccel = 60,
            firstSpecimenWait = 0,
            minFirstProfileAccel = -45,
            clampWaitBeforeOverhangSpecimen = 0.2,
            retractAfterOverhangSpecimenWait = 0.6,
            setupOverhangSpecimenWait = 0.5,
            scoreToRetractWait = 0.7,
            sleepSecondsBeforeUnclampFirst = 1,
            sleepSecondsBeforeUnclampSecond = 2.3,
            sleepSecondsBeforeUnclampThird = 2.1,
            sleepSecondsBeforeUnclampFourth = 2,
            sleepSecondsBeforeUnclampFifth = 2,

            secondSpecimenSleepBeforeSetup = 0.5,
            lastThreeSleepBeforeSetup = 0.5,
            bumpSpecimenVelConstraint = 20;

    @Override
    protected void configure() {
        super.configure();
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
//         Get gamepad 1 button input and save "right" and "red" booleans for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (gamepadEx1.wasJustPressed(A)) is5plus0 = !is5plus0;
            if (gamepadEx1.wasJustPressed(Y)) usePartnerSpec = !usePartnerSpec;
            if (gamepadEx1.wasJustPressed(B)) Common.IS_RED = true;
            if (gamepadEx1.wasJustPressed(X)) Common.IS_RED = false;
            mTelemetry.addLine("| B - Red alliance | X - Blue alliance |");
            mTelemetry.addLine("| A - Toggle 5+0 | Y - Toggle using partner specimen |");
            mTelemetry.addLine();
            mTelemetry.addLine("Selected alliance : " + (Common.IS_RED ? "Red" : "Blue"));
            mTelemetry.addLine("5+0 : " + (is5plus0 ? "enabled" : "disabled"));
            mTelemetry.addLine("Using partner specimen : " + (usePartnerSpec ? "enabled" : "disabled"));
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }
    }

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(startingPositionX, startingPositionY, Math.toRadians(270));
    }

    @Override
    protected void onInit() {
        super.onInit();
        robot.arm.setArmAngle(Arm.ArmAngle.CHAMBER_FRONT_SETUP);
        robot.arm.setWristAngle(Arm.WristAngle.FRONT_WALL_SPECIMEN_SCORE);
        robot.claw.setAngle(Claw.ClawAngles.CLAMPED);
        robot.setCurrentState(Robot.State.FRONT_WALL_PICKUP);
//        robot.intake.setTargetV4BAngle(Intake.V4BAngle.UP);

        robot.arm.run(false);
        robot.claw.run();
//        robot.intake.run();
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
                .strafeToSplineHeading(new Vector2d(parkX, parkY), Math.toRadians(315), (pose2dDual, posePath, v) -> parkVelocityConstraint);
        return builder;
    }

    private TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder, double offsetX, double offsetY, boolean doPark, double sleepSecondsBeforeSetup, double sleepSecondsBeforeUnclamp) {
        // Scoring
        builder = builder
                .setTangent(Math.toRadians(170))
                .afterTime(sleepSecondsBeforeUnclamp, new SequentialAction(
                        RobotActions.scoreOverhangSpecimen(),
                        new SleepAction(scoreToRetractWait),
                        RobotActions.setupFrontWallPickup()
                ))
                .splineToConstantHeading(new Vector2d(5 + offsetX, scoreSpecimenY + offsetY), Math.toRadians(90), (pose2dDual, posePath, v) -> scoreSpecimenVelocityConstraint, new ProfileAccelConstraint(minScoreProfileAccel, maxScoreProfileAccel))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(wallPickupX, intakeSpecimenY), Math.toRadians(270), (pose2dDual, posePath, v) -> scoreSpecimenVelocityConstraint);

        // Setting up for the next cycle
        if (!doPark) {
            builder = builder
                    .afterTime(startBumpToClampTime, RobotActions.stableTakeAndSetupOverhangSpecimen(sleepSecondsBeforeSetup))
                    .lineToY(bumpSpecimen, ((pose2dDual, posePath, v) -> bumpSpecimenVelConstraint));
        }

        return builder;
    }

    private TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {

        builder = builder
                .splineToSplineHeading(new Pose2d(firstWallPickupX, intakeSpecimenY, Math.toRadians(270)), Math.toRadians(270))
                .afterTime(secondSpecimenStartBumpToClampTime, RobotActions.stableTakeAndSetupOverhangSpecimen(secondSpecimenSleepBeforeSetup))
                .splineToSplineHeading(new Pose2d(firstWallPickupX, bumpSecondSpecimen, Math.toRadians(270)), Math.toRadians(270));

        builder = scoreSpecimen(builder, secondSpecimenOffsetX, secondSpecimenOffsetY, false, lastThreeSleepBeforeSetup, sleepSecondsBeforeUnclampSecond);
        builder = scoreSpecimen(builder, thirdSpecimenOffsetX, thirdSpecimenOffsetY, false, lastThreeSleepBeforeSetup, sleepSecondsBeforeUnclampThird);
        builder = scoreSpecimen(builder, fourthSpecimenOffsetX, fourthSpecimenOffsetY, !is5plus0, lastThreeSleepBeforeSetup, sleepSecondsBeforeUnclampFourth);
        if (is5plus0)
            builder = scoreSpecimen(builder, fifthSpecimenOffsetX, fifthSpecimenOffsetY, true, 0, sleepSecondsBeforeUnclampFifth);

        return builder;
    }
    private TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        boolean do3rdSample = is5plus0 || !usePartnerSpec;
        builder = builder
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(35,-35), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(sample1X, startFirstSampleY), Math.toRadians(270), (pose2dDual, posePath, v) -> giveSampleVelocityConstraint)
//                .afterTime(giveFirstSampleSweeperWait, new SequentialAction(
//                        RobotActions.setSweeper(Sweeper.SweeperAngles.ACTIVE, secondSweeperSleep),
//                        RobotActions.setSweeper(Sweeper.SweeperAngles.RETRACTED, 0)
//                ))
                .afterTime(0, RobotActions.setupFrontWallPickup())
                .splineToLinearHeading(new Pose2d(giveSample1X, giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(120), (pose2dDual, posePath, v) -> giveSampleVelocityConstraint, new ProfileAccelConstraint(minProfileAccel, maxProfileAccel))
                .splineToConstantHeading(new Vector2d(sample2X, startSampleY), Math.toRadians(270), (pose2dDual, posePath, v) -> giveSampleVelocityConstraint, new ProfileAccelConstraint(minProfileAccel, maxProfileAccel))
//                .afterTime(giveSecondSampleSweeperWait, new SequentialAction(
//                        RobotActions.setSweeper(Sweeper.SweeperAngles.ACTIVE, secondSweeperSleep),
//                        RobotActions.setSweeper(Sweeper.SweeperAngles.RETRACTED, 0)
//                ))
                .splineToLinearHeading(new Pose2d((!do3rdSample ? 4 : 0) + giveSample2X, giveSample2Y, Math.toRadians(givingSampleAngle)), Math.toRadians(!do3rdSample ? 270 : 120), (pose2dDual, posePath, v) -> giveSampleVelocityConstraint, new ProfileAccelConstraint(minProfileAccel, maxProfileAccel));

        if (do3rdSample)
            builder = builder
                    .splineToConstantHeading(new Vector2d(sample3X, startSampleY), Math.toRadians(270), (pose2dDual, posePath, v) -> giveSampleVelocityConstraint, new ProfileAccelConstraint(minProfileAccel, maxProfileAccel))
                    .afterTime(0, new SequentialAction(
                            RobotActions.setSweeper(Sweeper.SweeperAngles.ACTIVE, thirdSweeperSleep),
                            RobotActions.setSweeper(Sweeper.SweeperAngles.RETRACTED, 0)
                    ))
                    .splineToLinearHeading(new Pose2d(giveSample3X, giveSample3Y, Math.toRadians(givingSampleAngle)), Math.toRadians(270), (pose2dDual, posePath, v) -> giveSampleVelocityConstraint, new ProfileAccelConstraint(minProfileAccel, maxProfileAccel));

        return builder;
    }

    private TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(0, RobotActions.takeAndSetupOverhangSpecimen())
                .afterTime(sleepSecondsBeforeUnclampFirst, new SequentialAction(
                        RobotActions.scoreOverhangSpecimen(),
                        new SleepAction(scoreToRetractWait),
                        RobotActions.retractToNeutral(0)
                ))
                .lineToY((scoreSpecimenY), (pose2dDual, posePath, v) -> scoreFirstSpecimenVelocityConstraint, new ProfileAccelConstraint(minFirstProfileAccel, maxProfileAccel));
        return builder;
    }


}
