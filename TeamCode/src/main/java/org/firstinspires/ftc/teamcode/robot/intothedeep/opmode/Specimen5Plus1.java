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
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Sweeper;

@Autonomous(name = "Specimen 5+1")
@Config
public class Specimen5Plus1 extends AbstractAuto {
    private boolean
            is5plus0 = true,
            usePartnerSpec = false;

    public static double
            intakeSampleVelocityConstraint = 160,
            startingPositionX = 7.375,
            startingPositionY = -62,
            scoreSpecimenY = -29.5,
            sampleX = 21,
            sampleY = -44.6,
            extendSleep = 0.2,
            secondSpecimenOffsetY = 7,
            thirdSpecimenOffsetY = 5,
            fourthSpecimenOffsetY = 5,
            fifthSpecimenOffsetY = 6,
            secondSpecimenOffsetX = -8,
            thirdSpecimenOffsetX = -5.5,
            fourthSpecimenOffsetX = -2.5,
            fifthSpecimenOffsetX = 3.5,
            sample1X = 47,
            sample2X = 55,
            sample3X = 62,
            startFirstSampleY = -12,
            startSampleY = -14,
            bumpSpecimen = -65.5,
            bumpSecondSpecimen = -63,
            intakeSpecimenY = -56,
            giveSample1X = sample1X - 4,
            giveSample2X = sample2X - 4,
            giveSample3X = sample3X,
            giveSampleY = -47,
            wallPickupX = 35,
            firstWallPickupX = 55,
            secondSweeperSleep = 0.3,
            thirdSweeperSleep = 0.4,
            startBumpToClampTime = 0.35,
            secondSpecimenStartBumpToClampTime = 0.15,
            givingSampleAngle = 270,
            setupFrontWallPickupWait = 0.2,
            scoreSpecimenVelocityConstraint = 200,
            scoreSampleVelocityConstraint = 160,
            specimenAutoBasketX = -47,
            specimenAutoBasketY = -50,
            transferWait = 0,
            sampleScoreWait = 0,
            giveSweeperWait = 0.5,
            parkX = 10,
            parkY = -50,
            parkVelocityConstraint = 180,
            firstSpecimenWait = 1;

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
        builder = scoreSample(builder);
//        builder = park(builder);

        return builder.build();
    }

    private TrajectoryActionBuilder park(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(extendSleep, new ParallelAction(
                        RobotActions.setExtendo(Extendo.Extension.EXTENDED,0),
                        RobotActions.setArm(Arm.ArmAngle.BASKET,0),
                        RobotActions.setWrist(Arm.WristAngle.BASKET,0)
                ))
                .strafeToSplineHeading(new Vector2d(parkX, parkY), Math.toRadians(315), (pose2dDual, posePath, v) -> parkVelocityConstraint);
        return builder;
    }

    private TrajectoryActionBuilder scoreSample(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(extendSleep, new ParallelAction(
                        RobotActions.setRollers(1, 0),
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                        RobotActions.setExtendo(Extendo.Extension.EXTENDED,0)
                ))

                .strafeToSplineHeading(new Vector2d(sampleX, sampleY), Math.toRadians(315), (pose2dDual, posePath, v) -> intakeSampleVelocityConstraint)
                .afterTime(transferWait, RobotActions.retractTransferAndSetupBasket())
                .setTangent(Math.toRadians(-135))
                .splineToSplineHeading(new Pose2d(specimenAutoBasketX, specimenAutoBasketY, Math.toRadians(0)), Math.toRadians(-135), (pose2dDual, posePath, v) -> scoreSampleVelocityConstraint)
                .afterTime(sampleScoreWait, RobotActions.scoreBasket());
//                .stopAndAdd(RobotActions.retractToNeutral(0));
        return builder;
    }

    private TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder, double offsetX, double offsetY, boolean doPark) {
         builder = builder
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(5 + offsetX, scoreSpecimenY + offsetY), Math.toRadians(90), (pose2dDual, posePath, v) -> scoreSpecimenVelocityConstraint)
//                .strafeToConstantHeading(new Vector2d(5 + offsetX, scoreSpecimenY))
                .stopAndAdd(RobotActions.scoreSpecimenFromFrontWallPickup());

         if (!doPark) {
             builder = builder
                     .afterTime(setupFrontWallPickupWait, RobotActions.setupFrontWallPickup())
                     .setTangent(Math.toRadians(270))
                     .splineToConstantHeading(new Vector2d(wallPickupX, intakeSpecimenY), Math.toRadians(270), (pose2dDual, posePath, v) -> scoreSpecimenVelocityConstraint)
                     .afterTime(startBumpToClampTime, RobotActions.takeSpecimenFromFrontWallPickup(true))
                     .lineToY(bumpSpecimen);
         }

         return builder;
    }

    private TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {

        builder = builder
                .splineToSplineHeading(new Pose2d(firstWallPickupX, intakeSpecimenY, Math.toRadians(270)), Math.toRadians(270))
                .afterTime(secondSpecimenStartBumpToClampTime, RobotActions.takeSpecimenFromFrontWallPickup(true))
                .splineToSplineHeading(new Pose2d(firstWallPickupX, bumpSecondSpecimen, Math.toRadians(270)), Math.toRadians(270));

        builder = scoreSpecimen(builder, secondSpecimenOffsetX, secondSpecimenOffsetY, false);
        builder = scoreSpecimen(builder, thirdSpecimenOffsetX, thirdSpecimenOffsetY, false);
        builder = scoreSpecimen(builder, fourthSpecimenOffsetX, fourthSpecimenOffsetY, !is5plus0);
        if (is5plus0)
            builder = scoreSpecimen(builder, fifthSpecimenOffsetX, fifthSpecimenOffsetY, true);

        return builder;
    }
    private TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        boolean do3rdSample = is5plus0 || !usePartnerSpec;
        builder = builder
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(35,-35), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(sample1X, startFirstSampleY), Math.toRadians(270))//, giveSampleVelConstraint)
                .afterTime(0, RobotActions.setupFrontWallPickup())
                .splineToLinearHeading(new Pose2d(giveSample1X, giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(120))
                .splineToConstantHeading(new Vector2d(sample2X, startSampleY), Math.toRadians(270))
                .afterTime(giveSweeperWait, new SequentialAction(
                        RobotActions.setSweeper(Sweeper.SweeperAngles.ACTIVE, secondSweeperSleep),
                        RobotActions.setSweeper(Sweeper.SweeperAngles.RETRACTED, 0)
                ))
                .splineToLinearHeading(new Pose2d((!do3rdSample ? 4 : 0) + giveSample2X, giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(!do3rdSample ? 270 : 120));

        if (do3rdSample)
            builder = builder
                    .splineToConstantHeading(new Vector2d(sample3X, startSampleY), Math.toRadians(270))
                    .afterTime(0, new SequentialAction(
                            RobotActions.setSweeper(Sweeper.SweeperAngles.ACTIVE, thirdSweeperSleep),
                            RobotActions.setSweeper(Sweeper.SweeperAngles.RETRACTED, 0)
                    ))
                    .splineToLinearHeading(new Pose2d(giveSample3X, giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(270));

        return builder;
    }

    private TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(0, RobotActions.takeSpecimenFromFrontWallPickup(false))
                .afterTime(firstSpecimenWait, RobotActions.scoreSpecimenFromFrontWallPickup())
                .lineToY(scoreSpecimenY);
        return builder;
    }


}
