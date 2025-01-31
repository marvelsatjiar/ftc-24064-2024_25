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
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Claw;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Sweeper;

@Autonomous(name = "Specimen Side")
@Config
public class SpecimenAuton extends AbstractAuto {
    private boolean
            is5plus0 = true,
            usePartnerSpec = false;
    public static double
            slowDownConstraint = 20,
            parkVelocityConstraint = 160,
            startingPositionX = 7.375,
            startingPositionY = -62,
            scoreSpecimenY = -30,
            parkX = 21,
            parkY = -44.6,
            parkExtendSleep = 0.2,
            secondSpecimenOffsetY = 2,
            fifthSpecimenOffsetY = 4.75,
            sample1X = 47,
            sample2X = 55,
            sample3X = 62,
            startSampleY = -14,
            slowDownStartY = -25,
            bumpSpecimen = -60,
            bumpSecondSpecimen = -62,
            intakeSpecimenY = -56,
            giveSample1X = sample1X - 4,
            giveSample2X = sample2X - 4,
            giveSample3X = sample3X,
            giveSampleY = -50,
            wallPickupX = 35,
            firstWallPickupX = 55,
            sweeperSleep = 1,
            startBumpToClampTime = 0.35,
            givingSampleAngle = 270,
            regularGivingConstraint = 40,
            setupFrontWallPickupWait = 0.2,
            bumpSpecimenVelocityConstraint = 15,
            scoreSpecimenVelocityConstraint = 160,
            firstSpecimenWait = 1.5;



    private static final VelConstraint giveSampleVelConstraint = (robotPose, path, disp) -> {
        if (robotPose.position.y.value() > slowDownStartY) {
            return slowDownConstraint;
        } else {
            return regularGivingConstraint;
        }
    };

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
        builder = park(builder);

        return builder.build();
    }

    private TrajectoryActionBuilder park(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(parkExtendSleep, new ParallelAction(
                        RobotActions.setExtendo(Extendo.Extension.EXTENDED,0),
                        RobotActions.setArm(Arm.ArmAngle.BASKET,0),
                        RobotActions.setWrist(Arm.WristAngle.BASKET,0)
                ))
                .strafeToSplineHeading(new Vector2d(parkX, parkY), Math.toRadians(315), (pose2dDual, posePath, v) -> parkVelocityConstraint);
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
                    .lineToY(bumpSpecimen, (pose2dDual, posePath, v) -> bumpSpecimenVelocityConstraint);
        }

        return builder;
    }

    private TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {

        builder = builder
                .splineToSplineHeading(new Pose2d(firstWallPickupX, intakeSpecimenY, Math.toRadians(270)), Math.toRadians(270))
                .afterTime(startBumpToClampTime, RobotActions.takeSpecimenFromFrontWallPickup(true))
                .splineToSplineHeading(new Pose2d(firstWallPickupX, bumpSecondSpecimen, Math.toRadians(270)), Math.toRadians(270), (pose2dDual, posePath, v) -> bumpSpecimenVelocityConstraint);

        builder = scoreSpecimen(builder, -7.5, secondSpecimenOffsetY, false);
        builder = scoreSpecimen(builder, -5, 2, false);
        builder = scoreSpecimen(builder, -2.5, 3.75, !is5plus0);
        if (is5plus0)
            builder = scoreSpecimen(builder, 4, fifthSpecimenOffsetY, true);

        return builder;
    }
    private TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        boolean do3rdSample = is5plus0 || !usePartnerSpec;
        builder = builder
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(35,-35),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(sample1X, startSampleY), Math.toRadians(270))//, giveSampleVelConstraint)
                .afterTime(0, RobotActions.setupFrontWallPickup())
                .splineToLinearHeading(new Pose2d(giveSample1X, giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(120))
                .splineToConstantHeading(new Vector2d(sample2X, startSampleY), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d((!do3rdSample ? 4 : 0) + giveSample2X,giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(!do3rdSample ? 270 : 120));

        if (do3rdSample)
            builder = builder
                    .splineToConstantHeading(new Vector2d(sample3X, startSampleY), Math.toRadians(270))
                    .afterTime(0, new SequentialAction(
                            RobotActions.setSweeper(Sweeper.SweeperAngles.ACTIVE, sweeperSleep),
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
