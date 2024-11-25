package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;


@Autonomous(name ="Sample Side")
@Config
public class SampleAuto extends AbstractAuto {
    public static double
            scoreSpecimenY = -29.5,
            waitToScoreSample1 = 4,
            waitToScoreSample2 = 4,
            robotAngle = 97,
            xSample1 = -44,
            ySample1 = -34,
            xSample2 = -58,
            ySample2 = -34,
            xBasket = -55,
            yBasket = -55;

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(-8,-63,Math.toRadians(-270));
    }

    @Override
    protected Action onRun() {
        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());
        builder = scoreSpecimen(builder);
        builder = scoreSamples(builder);
//        builder = samplePark(builder);

        return builder.build();
    }

    private TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(0.0, new ParallelAction(
                        RobotActions.setupChamberFromFront(),
                        RobotActions.setClaw(true, 0.0)
                ))
                .lineToY(scoreSpecimenY)
                .stopAndAdd( RobotActions.scoreChamberFromFrontAndRetract());
        return builder;
    }

    private TrajectoryActionBuilder scoreSamples(TrajectoryActionBuilder builder) {
        builder = builder
                // Moving to 1st sample
                .setTangent(Math.toRadians(270))
                .afterTime(1, RobotActions.setV4B(Intake.V4BAngle.HOVERING, 0))
                .afterTime(2, new SequentialAction(
                        new ParallelAction(
                                RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                                RobotActions.setRollers(1, 2)
                        ),
                        RobotActions.transferToClaw(),
                        RobotActions.setupScoreBasket(true)
                ))
                .splineToSplineHeading(new Pose2d(xSample1, ySample1, Math.toRadians(-235)), Math.toRadians(180))
                // 1st sample intaking

                // Moving to basket
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(xBasket, yBasket, Math.toRadians(45)), Math.toRadians(-135))
                .waitSeconds(waitToScoreSample1)
//                // Basket
                .afterTime(0, RobotActions.scoreBasketAndRetract())
                // Moving to 2nd sample
                .setTangent(Math.toRadians(110))
                .afterTime(0, RobotActions.setV4B(Intake.V4BAngle.HOVERING, 0))
                .splineToLinearHeading(new Pose2d(xSample2, ySample2, Math.toRadians(robotAngle)),Math.toRadians(110))
                // 2nd sample intaking
                .afterTime(0, new SequentialAction(
                    new ParallelAction(
                            RobotActions.setV4B(Intake.V4BAngle.DOWN, 0),
                            RobotActions.setRollers(1, 2)
                    ),
                    RobotActions.transferToClaw(),
                    RobotActions.setupScoreBasket(true)
                ))
                // Moving to basket
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(xBasket, yBasket, Math.toRadians(45)),Math.toRadians(290))
                .waitSeconds(waitToScoreSample2)
                // Basket
                .afterTime(0, RobotActions.scoreBasketAndRetract());
//                // Moving to 3rd sample
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-54,-46,Math.toRadians(127)),Math.toRadians(127))
//                // 3rd sample intaking
//                .stopAndAdd(new SequentialAction(
//                        RobotActions.extendIntake(Extendo.Extension.ONE_FOURTH),
//                        RobotActions.setRollers(1, 0.5)
//                ))
//                .afterTime(0.0, new SequentialAction(
//                        RobotActions.transferToClaw(),
//                        RobotActions.setupScoreBasket(true)
//
//                ))
//                // Moving to basket
//                .setTangent(Math.toRadians(255))
//                .splineToLinearHeading(new Pose2d(-56,-56,Math.toRadians(45)),Math.toRadians(255))
//                // Basket
//                .afterTime(0.0, RobotActions.scoreBasketAndRetract(true));
        return builder;
    }

    private TrajectoryActionBuilder samplePark(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-25,-10,Math.toRadians(180)),Math.toRadians(0));
        return builder;
    }
}
