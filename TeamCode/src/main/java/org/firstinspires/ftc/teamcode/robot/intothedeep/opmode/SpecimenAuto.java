package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Claw;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;

@Autonomous(name = "Specimen Side")
@Config
public class SpecimenAuto extends AbstractAuto {
    private boolean
            is5plus0 = false,
            usePartnerSpec = false;
    public static double
            slowDownConstraint = 25,
            startingPositionX = 7.375,
            startingPositionY = -62,
            scoreSpecimenY = -31,
            sample1X = 48,
            sample2X = 58,
            sample3X = 63,
            startSampleY = -14,
            bumpSpecimen = -64,
            intakeSpecimenY = -60,
            giveSample1X = sample1X - 4,
            giveSample2X = sample2X - 4,
            giveSample3X = sample3X,
            giveSampleY = -50,
            wallPickupX = 35,
            firstWallPickupX = 58,
            startBumpToClampTime = 0,
            givingSampleAngle = 270;

    private static final VelConstraint giveSampleVelConstraint = (robotPose, path, disp) -> {
        if (robotPose.position.y.value() > -17) {
            return slowDownConstraint;
        } else {
            return 60.0;
        }
    };

    @Override
    protected void configure() {
        super.configure();
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
//         Get gamepad 1 button input and save "right" and "red" booleans for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (gamepadEx1.wasJustPressed(B)) is5plus0 = !is5plus0;
            if (gamepadEx1.wasJustPressed(X)) usePartnerSpec = !usePartnerSpec;
            mTelemetry.addLine("| B - Toggle 5+0 | X - Toggle using partner specimen |");
            mTelemetry.addLine();
            mTelemetry.addLine("5+0 : " + (is5plus0 ? "enabled" : "disabled"));
            mTelemetry.addLine("Using partner specimen : " + (usePartnerSpec ? "enabled" : "disabled"));
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }
    }

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(startingPositionX, startingPositionY, Math.toRadians(-270));
    }

    @Override
    protected void onInit() {
        super.onInit();
        robot.arm.setArmAngle(Arm.ArmAngle.CHAMBER_FRONT_SETUP);
        robot.arm.setWristAngle(Arm.WristAngle.CHAMBER_FRONT);
        robot.claw.setAngle(Claw.ClawAngles.CLAMPED);
        robot.intake.setTargetV4BAngle(Intake.V4BAngle.UP);

        robot.arm.run(false);
        robot.claw.run();
        robot.intake.run();
    }

    @Override
    protected Action onRun() {
        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());

        builder = scoreFirstSpecimen(builder);
        builder = giveSamples(builder);
        builder = scoreAllSpecimens(builder);

        return builder.build();
    }

    private TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder, double offset, boolean doPark) {
         builder = builder
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(5 + offset, scoreSpecimenY), Math.toRadians(90))
                .stopAndAdd(RobotActions.scoreSpecimenFromFrontWallPickup());

         if (!doPark) {
             builder = builder.afterTime(0.5, RobotActions.setupFrontWallPickup());
         }

         builder = builder
                .setTangent(Math.toRadians(270))
                .splineTo(new Vector2d(wallPickupX, intakeSpecimenY), Math.toRadians(270));

         if (!doPark) {
             builder = builder.afterTime(startBumpToClampTime, RobotActions.takeSpecimenFromFrontWallPickup());
         }

         builder = builder.lineToY(bumpSpecimen);

         return builder;
    }

    private TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {

        builder = builder
                .splineToSplineHeading(new Pose2d(firstWallPickupX, intakeSpecimenY, Math.toRadians(270)), Math.toRadians(270))
                .afterTime(startBumpToClampTime, RobotActions.takeSpecimenFromFrontWallPickup())
                .splineToSplineHeading(new Pose2d(firstWallPickupX, bumpSpecimen, Math.toRadians(270)), Math.toRadians(270));

        builder = scoreSpecimen(builder, 0, false);
        builder = scoreSpecimen(builder, -2, false);
        builder = scoreSpecimen(builder, -4, !is5plus0);
        if (is5plus0)
            builder = scoreSpecimen(builder, -6, true);

        return builder;
    }
    private TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        boolean do3rdSample = is5plus0 || !usePartnerSpec;
        builder = builder
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(33,-35),Math.toRadians(90))
                .afterTime(0, RobotActions.setV4B(Intake.V4BAngle.HOVERING, 0))
                .splineToSplineHeading(new Pose2d(sample1X, startSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(270), giveSampleVelConstraint)
                .afterTime(0, RobotActions.setupFrontWallPickup())
                .splineToLinearHeading(new Pose2d(giveSample1X, giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(120))
                .splineToSplineHeading(new Pose2d(sample2X, startSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(270), giveSampleVelConstraint)
                .splineToLinearHeading(new Pose2d((!do3rdSample ? 4 : 0) + giveSample2X,giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(!do3rdSample ? 270 : 120));

        if (do3rdSample)
            builder = builder
                    .splineToSplineHeading(new Pose2d(sample3X, startSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(270), giveSampleVelConstraint)
                    .splineToLinearHeading(new Pose2d(giveSample3X, giveSampleY, Math.toRadians(givingSampleAngle)), Math.toRadians(270));

        return builder.afterTime(0, RobotActions.setV4B(Intake.V4BAngle.UP, 0));
    }

    private TrajectoryActionBuilder scoreFirstSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(0, RobotActions.takeSpecimenFromFrontWallPickup())
                .lineToY(scoreSpecimenY)
                .stopAndAdd(RobotActions.scoreSpecimenFromFrontWallPickup());
        return builder;
    }
}
