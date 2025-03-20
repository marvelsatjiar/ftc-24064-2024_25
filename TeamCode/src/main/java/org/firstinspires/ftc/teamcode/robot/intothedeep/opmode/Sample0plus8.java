package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
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


@Autonomous(name ="Sample 0+8")
@Config
public class Sample0plus8 extends AbstractAuto {

    public static class Positions {
        public double
                startingPositionX = -39,
                startingPositionY = -63.375,
                xBasket1 = -58,
                yBasket1 = -51,
                xBasket2 = -62,
                yBasket2 = -51,
                xIntakeSample3 = -51,
                yIntakeSample3 = -46,
                subX = -25,
                subY = -12,
                xBasketSub = -53,
                yBasketSub = -53,
                sample5thOffset = 1,
                sample6thOffset = 1,
                sample7thOffset = 1,
                sample8thOffset = 1;
    }

    public static class Headings {
        public double
                intake1stSampleAngle = 68,
                intake2ndSampleAngle = 82,
                intake3rdSampleAngle = 125,
                basketAngle = 45;
    }

    public static class Timings {
        public double
                waitBefore2ndTransfer = 2,
                waitBefore3rdTransfer = 2,
                sleep3rdBeforeExtending = 3,
                sleep4thBeforeExtending = 3,
                intake4thDelayAfter3rd = 2,
                waitBefore3rdRetract = 2,
                sleepBefore4thScore = 2
        ;

    }


    public static class Miscellaneous {

    }

    public static Positions POS = new Positions();
    public static Headings HEAD = new Headings();
    public static Timings TIME = new Timings();
    public static Miscellaneous MISC = new Miscellaneous();


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
        return new Pose2d(POS.startingPositionX, POS.startingPositionY, Math.toRadians(0));
    }

    @Override
    protected void onInit() {
        robot.claw.setAngle(Claw.ClawAngles.SAMPLE_CLAMPED);
        robot.arm.setArmAngle(Arm.ArmAngle.NEUTRAL);
        robot.arm.setWristAngle(Arm.WristAngle.BASKET);
        robot.setCurrentState(Robot.State.TRANSFERRED);

        robot.arm.run();
        robot.claw.run();
    }

    @Override
    protected Action onRun() {
        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());
        builder = scoreFirstFourSamples(builder);
        builder = scoreAllSubSamples(builder);

        return builder.build();
    }


    private static TrajectoryActionBuilder scoreFirstFourSamples(TrajectoryActionBuilder builder) {
        builder = builder

                // Setup Score 1st Sample Objectively + Extends for 2nd Objectively
                .afterTime(0, new ParallelAction(
                        RobotActions.setupBasket(true),
                        RobotActions.setExtendo(100, 0)
                ))
                .strafeToLinearHeading(new Vector2d(POS.xBasket1, POS.yBasket1), Math.toRadians(HEAD.intake1stSampleAngle))

                // Score 1st Sample + Intakes 2nd Objectively TODO waitBefore2ndTransfer
                .stopAndAdd(new ParallelAction(
                        new SequentialAction(
                                RobotActions.scoreBasket(),
                                RobotActions.retractAfterScoreBasket()
                        ),
                        RobotActions.setExtendo(132, 0),
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                        RobotActions.setRollers(1, TIME.waitBefore2ndTransfer)
                ))


                // Transfer & Setup 2nd Objectively + Extend for 3rd Objectively TODO sleep3rdBeforeExtending
                .afterTime(0, new ParallelAction(
                        RobotActions.retractTransferAndSetupBasket(),
                        new SequentialAction(
                                new SleepAction(TIME.sleep3rdBeforeExtending),
                                RobotActions.setExtendo(100, 0)
                        )
                ))

                .strafeToLinearHeading(new Vector2d(POS.xBasket2, POS.yBasket2), Math.toRadians(HEAD.intake2ndSampleAngle))

                // Score 2nd + Intake 3rd Objectively TODO waitBefore3rdTransfer
                .stopAndAdd(new ParallelAction(
                        new SequentialAction(
                                RobotActions.scoreBasket(),
                                RobotActions.retractAfterScoreBasket()
                        ),
                        RobotActions.setExtendo(132, 0),
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                        RobotActions.setRollers(1, TIME.waitBefore3rdTransfer)
                ))
                // Transfer, Setup, & Scores 3rd Objectively + Extend for 4th Objectively TODO sleep4thBeforeExtending
                .stopAndAdd(new ParallelAction(
                        new SequentialAction(
                                RobotActions.retractTransferAndSetupBasket(),
                                RobotActions.scoreBasket()
                        ),
                        new SequentialAction(
                                new SleepAction(TIME.sleep4thBeforeExtending),
                                RobotActions.setExtendo(100, 0)
                        )
                ))
                // Retract after 3rd Scored Objectively
                .afterTime(TIME.waitBefore3rdRetract, RobotActions.retractAfterScoreBasket())

                // Moving to & Intaking 4th sample Objectively
                .afterTime(TIME.intake4thDelayAfter3rd, new ParallelAction(
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                        RobotActions.setExtendo(132, 0),
                        RobotActions.setRollers(1, 0)
                ))

                .strafeToLinearHeading(new Vector2d(POS.xIntakeSample3, POS.yIntakeSample3), Math.toRadians(HEAD.intake3rdSampleAngle))
                .waitSeconds(TIME.sleepBefore4thScore)

                //Transfer while moving to Score 4th Sample Objectively
                .afterTime(0, RobotActions.retractTransferAndSetupBasket())
                .strafeToLinearHeading(new Vector2d(POS.xBasketSub, POS.yBasketSub), Math.toRadians(HEAD.basketAngle))
                //Scoring 4th Sample Objectively
                .stopAndAdd(RobotActions.scoreBasket());
        return builder;
    }

    private static TrajectoryActionBuilder scoreAllSubSamples(TrajectoryActionBuilder builder) {
        builder = scoreSubSamples(builder, POS.sample5thOffset);
        builder = scoreSubSamples(builder, POS.sample6thOffset);
        builder = scoreSubSamples(builder, POS.sample7thOffset);
        builder = scoreSubSamples(builder, POS.sample8thOffset);
        builder = builder.afterTime(0, RobotActions.retractAfterScoreBasket());
        return builder;

    }

    private static TrajectoryActionBuilder scoreSubSamples(TrajectoryActionBuilder builder, double offsetY) {
        builder = builder
                // Retract while moving to Sub from Scoring
                .afterTime(0.5, RobotActions.retractAfterScoreBasket())
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(POS.subX, POS.subY + offsetY, Math.toRadians(0)), Math.toRadians(0))
                // Sweeping + Intaking Sub Sample TODO vision + remove hardcoded waits
                .stopAndAdd(new ParallelAction(
                        new SequentialAction(
                                RobotActions.setSweeper(Sweeper.SweeperAngles.ACTIVE, 1),
                                RobotActions.setSweeper(Sweeper.SweeperAngles.RETRACTED, 0)
                        ),
                        new SequentialAction(
                                RobotActions.setExtendo(70, 0.5),
                                RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                                RobotActions.setRollers(1, 0),
                                RobotActions.setExtendo(120, 0.5)
                        )
                ))
                // Transfer & Setup after Intaking Sub Sample while Moving to Basket
                .afterTime(0, RobotActions.retractTransferAndSetupBasket())
                .setTangent(Math.toRadians(180))
                // Score Sub Sample
                .splineToLinearHeading(new Pose2d(POS.xBasketSub, POS.yBasketSub, Math.toRadians(HEAD.basketAngle)), Math.toRadians(-135))
                .stopAndAdd(RobotActions.scoreBasket());
        return builder;
    }



}
