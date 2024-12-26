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
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Claw;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;


@Autonomous(name ="Sample Side")
@Config
public class SampleAuto extends AbstractAuto {
    public static double
            bumpSample = -34,
            startingPositionX = -31.85,
            startingPositionY = -63.375,
            scoreSpecimenY = -29,
            scoreSpecimenX = -4,
            waitToIntakeSample1 = 0,
            waitToIntakeSample = 0.5,
            waitToScoreSample3 = 4,
            robotAngle = 97,
            thirdSampleangle = 155,
            headingSample1 = 70,
            xSample1 = -54.0,
            ySample1 = -37.6,
            xSample2 = -57.5,
            ySample2 = -36,
            xSample3 = -62.3,
            ySample3 = -28,
            xBasket1 = -56,
            xBasket2 = -53.5,
            xBasket3 = -54,
            xBasket4 = -54,
            scoreWait = 2,
            intakeTime = 1,
            yBasket1 = -56,
            yBasket2 = -54.5,
            yBasket3 = -54,
            yBasket4 = -54.5;

    @Override
    protected void configure() {
        super.configure();
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
//         Get gamepad 1 button input and save "right" and "red" booleans for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (gamepadEx1.wasJustPressed(B)) Common.IS_RED = true;
            if (gamepadEx1.wasJustPressed(X)) Common.IS_RED = false;
            mTelemetry.addLine("| B - Red alliance | X - Blue alliance |");
            mTelemetry.addLine();
            mTelemetry.addLine("Selected alliance : " + (Common.IS_RED ? "Red" : "Blue"));
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }
    }


    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(startingPositionX, startingPositionY, Math.toRadians(0));
    }

    @Override
    protected void onInit() {
        robot.claw.setAngle(Claw.ClawAngles.CLAMPED);
        robot.arm.setArmAngle(Arm.ArmAngle.COLLECTING);
        robot.arm.setWristAngle(Arm.WristAngle.TRANSFERRED);
        robot.setCurrentState(Robot.State.TRANSFERRED);

        robot.arm.run(false);
        robot.claw.run();
    }

    @Override
    protected Action onRun() {
        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());
//        builder = scoreSpecimen(builder);
        builder = scoreSamples(builder);
        builder = samplePark(builder);

        return builder.build();
    }

    private TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder) {
        builder = builder

                .afterTime(0.0, new ParallelAction(
                        RobotActions.setupChamberFromFront(),
                        RobotActions.setClaw(Claw.ClawAngles.CLAMPED, 0.0)
                ))
                .splineToConstantHeading(new Vector2d(scoreSpecimenX, scoreSpecimenY), Math.toRadians(90))
                .stopAndAdd( RobotActions.scoreChamberFromFrontAndRetract());
        return builder;
    }

    private TrajectoryActionBuilder scoreSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(180))
                .afterTime(0, RobotActions.setupScoreBasket(true))
                .splineToLinearHeading(new Pose2d(xBasket1, yBasket1, Math.toRadians(45)), Math.toRadians(-135))
//                .waitSeconds(waitToScoreSample1)
                .stopAndAdd(new SequentialAction(
                        RobotActions.scoreBasket(),
                        RobotActions.setV4B(Intake.V4BAngle.HOVERING, 1),
                        RobotActions.retractToNeutral(0)
                        ))
                .afterTime(waitToIntakeSample1,
                        new ParallelAction(
                                RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                                RobotActions.setRollers(1, 0)

                ))
                .splineToLinearHeading(new Pose2d(xSample1, ySample1, Math.toRadians(headingSample1)), Math.toRadians(45))
                .lineToY(bumpSample)
                .afterTime(intakeTime, new SequentialAction(
                        RobotActions.transferToClaw(),
                        new SleepAction(1),
                        RobotActions.setupScoreBasket(true),
                        RobotActions.scoreBasket(),
                        RobotActions.setV4B(Intake.V4BAngle.HOVERING, scoreWait)
                ))
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(xBasket2, yBasket2, Math.toRadians(45)), Math.toRadians(-45))
//                the above is a working 0+2
                .waitSeconds(waitToScoreSample3)
                .afterTime(waitToIntakeSample,
                        new ParallelAction(
                                RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                                RobotActions.setRollers(1, 0),
                                RobotActions.retractToNeutral(0)
                        ))
                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(new Pose2d(xSample2, ySample2, Math.toRadians(robotAngle)),Math.toRadians(110))
                .lineToY(bumpSample)
                .afterTime(intakeTime, new SequentialAction(
                        RobotActions.transferToClaw(),
                        new SleepAction(1),
                        RobotActions.setupScoreBasket(true),
                        RobotActions.scoreBasket(),
                        RobotActions.setV4B(Intake.V4BAngle.HOVERING, scoreWait)
                ))
                .setTangent(Math.toRadians(-110))
                .splineToLinearHeading(new Pose2d(xBasket3, yBasket3, Math.toRadians(45)),Math.toRadians(290))
                .waitSeconds(waitToScoreSample3)
                .afterTime(waitToIntakeSample,
                        new ParallelAction(
                                RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                                RobotActions.setRollers(1, 0),
                                RobotActions.retractToNeutral(0)
                        ))
                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(new Pose2d(xSample3, ySample3, Math.toRadians(thirdSampleangle)),Math.toRadians(110))
                .lineToY(bumpSample)
                .afterTime(intakeTime, new SequentialAction(
                        RobotActions.transferToClaw(),
                        new SleepAction(1),
                        RobotActions.setupScoreBasket(true),
                        RobotActions.scoreBasket(),
                        RobotActions.setV4B(Intake.V4BAngle.HOVERING, scoreWait)
                ))
                .setTangent(Math.toRadians(-110))
                .splineToLinearHeading(new Pose2d(xBasket4, yBasket4, Math.toRadians(45)),Math.toRadians(290))
                .waitSeconds(waitToScoreSample3)
                .afterTime(0, RobotActions.retractToNeutral(0));
        return builder;
    }

    private TrajectoryActionBuilder samplePark(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-25,-10,Math.toRadians(180)),Math.toRadians(0));
        return builder;
    }
}
