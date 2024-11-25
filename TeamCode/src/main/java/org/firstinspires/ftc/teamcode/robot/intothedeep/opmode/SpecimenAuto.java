package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;

@Autonomous(name = "Specimen Side")
public class SpecimenAuto extends AbstractAuto {
    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(10,-63,Math.toRadians(-270));
    }

    @Override
    protected Action onRun() {
        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());

        builder = scoreSpecimen(builder);
        builder = giveSamples(builder);
        builder = scoreAllSpecimens(builder);
        builder = park(builder);

        return builder.build();
    }

    private TrajectoryActionBuilder park(TrajectoryActionBuilder builder) {
        builder = builder
                .lineToYLinearHeading(-48,Math.toRadians(-40));

        return builder;
    }

    private TrajectoryActionBuilder scoreAllSpecimens(TrajectoryActionBuilder builder) {
        builder = builder
                // Turning to intake 1st Specimen
                .setTangent(Math.toRadians(180))
                .stopAndAdd(new SequentialAction(RobotActions.setRollers(1, 0.5)))
                .afterTime(0.0, new SequentialAction(
                        RobotActions.transferToClaw(),
                        RobotActions.setupChamberFromBack()
                ))
                // Moving to chamber
                .splineToConstantHeading(new Vector2d(26,-38),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(7,-36),Math.toRadians(180))
                // Scoring 1st Specimen
                .stopAndAdd( RobotActions.scoreChamberFromFrontAndRetract())
                .afterTime(0.0, RobotActions.extendIntake(Extendo.Extension.ONE_FOURTH))
                // Moving to intake 2nd Specimen
                .setTangent(-45)
                .lineToYLinearHeading(-45,Math.toRadians(-40))
                .stopAndAdd(RobotActions.setRollers(1, 0.5))
                .afterTime(0.0, new SequentialAction(
                        RobotActions.transferToClaw(),
                        RobotActions.setupChamberFromBack()
                ))
                // Scoring 2nd Specimen
                .lineToYLinearHeading(-36,Math.toRadians(270))
                .stopAndAdd( RobotActions.scoreChamberFromFrontAndRetract())
                // Moving to intake 3rd Specimen
                .afterTime(0.0, RobotActions.extendIntake(Extendo.Extension.ONE_FOURTH))
                .lineToYLinearHeading(-45,Math.toRadians(-40))
                .stopAndAdd(RobotActions.setRollers(1, 0.5))
                .afterTime(0.0, new SequentialAction(
                        RobotActions.transferToClaw(),
                        RobotActions.setupChamberFromBack()
                ))
                // Scoring 3rd Specimen
                .lineToYLinearHeading(-36,Math.toRadians(270))
                .stopAndAdd( RobotActions.scoreChamberFromFrontAndRetract());
//                .lineToYLinearHeading(-45,Math.toRadians(-40))
//                .lineToYLinearHeading(-36,Math.toRadians(270));
        return builder;
    }

    private TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
                //moving to first sample
                .setTangent(Math.toRadians(-20))
                .splineToConstantHeading(new Vector2d(26,-38),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(35,-36,Math.toRadians(35)),Math.toRadians(35))
                // Intake 1st sample
                .stopAndAdd(new SequentialAction(
                        RobotActions.extendIntake(Extendo.Extension.ONE_FOURTH),
                        RobotActions.setRollers(1, 0.5)
                ))
                .turn(Math.toRadians(-70))
                .afterTime(0.0, new SequentialAction(
                        RobotActions.extendIntake(Extendo.Extension.EXTENDED),
                        RobotActions.setRollers(-1,0.2),
                        RobotActions.retractToNeutral(0.5)

                ))
                // Give Sample 1
                .splineToSplineHeading(new Pose2d(47,-36,Math.toRadians(35)),Math.toRadians(35))
                // Sample 2 intake
                .stopAndAdd(new SequentialAction(
                        RobotActions.extendIntake(Extendo.Extension.ONE_FOURTH),
                        RobotActions.setRollers(1, 0.5)
                ))
                .turn(Math.toRadians(-80))
                .afterTime(0.0, new SequentialAction(
                        RobotActions.extendIntake(Extendo.Extension.EXTENDED),
                        RobotActions.setRollers(-1,0.2),
                        RobotActions.retractToNeutral(0.5)

                ))

                // Give Sample 2
                .turn(Math.toRadians(65))
                .stopAndAdd(new SequentialAction(
                        RobotActions.extendIntake(Extendo.Extension.ONE_FOURTH),
                        RobotActions.setRollers(1, 0.5)
                ))
                // Intake sample 3
                .turn(Math.toRadians(-110))
                .afterTime(0.0, new SequentialAction(
                        RobotActions.extendIntake(Extendo.Extension.EXTENDED),
                        RobotActions.setRollers(-1,0.2)
                ));
                // Given Sample 3;
        return builder;
    }

    private TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(0.0, RobotActions.setupChamberFromFront())
                .lineToY(-35)
                .stopAndAdd( RobotActions.scoreChamberFromFrontAndRetract());
        return builder;
    }
}
