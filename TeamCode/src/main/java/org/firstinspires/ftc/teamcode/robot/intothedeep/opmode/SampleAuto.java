package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name ="Sample Side")
public class SampleAuto extends AbstractAuto {
    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(-10,-63,Math.toRadians(-270));
    }

    @Override
    protected Action onRun() {
        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());
        builder = scoreSpecimen(builder);
        builder = scoreSamples(builder);
        builder = samplePark(builder);

        return builder.build();
    }

    private TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .lineToY(-35);
        return builder;
    }

    private TrajectoryActionBuilder scoreSamples(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-48,-45,Math.toRadians(-270)),Math.toRadians(180))
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-56,-56,Math.toRadians(45)),Math.toRadians(-135))
                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(new Pose2d(-58,-50,Math.toRadians(90)),Math.toRadians(110))
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(-56,-56,Math.toRadians(45)),Math.toRadians(290))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-54,-46,Math.toRadians(127)),Math.toRadians(127))
                .setTangent(Math.toRadians(255))
                .splineToLinearHeading(new Pose2d(-56,-56,Math.toRadians(45)),Math.toRadians(255));
        return builder;
    }

    private TrajectoryActionBuilder samplePark(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-25,-10,Math.toRadians(180)),Math.toRadians(0));
        return builder;
    }
}
