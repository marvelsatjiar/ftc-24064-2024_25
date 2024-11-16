package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(26,-38),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(7,-36),Math.toRadians(180))
                .setTangent(-45)
                .lineToYLinearHeading(-45,Math.toRadians(-40))
                .lineToYLinearHeading(-36,Math.toRadians(270))
                .lineToYLinearHeading(-45,Math.toRadians(-40))
                .lineToYLinearHeading(-36,Math.toRadians(270))
                .lineToYLinearHeading(-45,Math.toRadians(-40))
                .lineToYLinearHeading(-36,Math.toRadians(270));
        return builder;
    }

    private TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(-20))
                .splineToConstantHeading(new Vector2d(26,-38),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(35,-36,Math.toRadians(35)),Math.toRadians(35))
                //extend intake ~5in
                .turn(Math.toRadians(-70))
                //give sample 2 below
                .splineToSplineHeading(new Pose2d(47,-36,Math.toRadians(35)),Math.toRadians(35))
                .turn(Math.toRadians(-80))
                //give sample 3 below
                .turn(Math.toRadians(65))
                .turn(Math.toRadians(-110));
        return builder;
    }

    private TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .lineToY(-35);
        return builder;
    }
}
