package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Claw;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.RobotActions;

@Autonomous(name = "Specimen Side")
@Config
public class SpecimenAuto extends AbstractAuto {
    public static double
            scoreSpecimenY = -29,
            firstSampleAngle = 35,
            xSample1 = 38,
            ySample1 = -36,
            xSample2 = 46,
            ySample2 = -35,
            xSample3 = 46,
            ySample3 = -40.5,
            bumpSample = -33.5,
            intakeSampleAngle3 = 29.25,
            extendoAngleSample3 = 85,
            bumpSample3 = -35.5,
            retractExtendoWait = 0.3,
            xSubmersibleSpecimen = 6,
            ySubmersibleSpecimen = -30,
            xintakeSpecimen = 16,
            bumpSpecimen = 19,
            yintakeSpecimen = -46,
            transferSpecimenToClawWait = 0.5,
            rollerIntakeSeconds = 1,
            setupChamberFromBackWait = 1,
            unclampAfterTimeWait = 0,
            UnclampSpecimenWait = 0.5;

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(8,-63,Math.toRadians(-270));
    }

    @Override
    protected Action onRun() {
        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());

        builder = scoreSpecimen(builder);
        builder = giveSamples(builder);
        builder = scoreAllSpecimens(builder);
//        builder = park(builder);

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
                .splineToLinearHeading(new Pose2d(xintakeSpecimen,yintakeSpecimen,Math.toRadians(-45)), Math.toRadians(180))
                .stopAndAdd( new SequentialAction(
                        RobotActions.setV4B(Intake.V4BAngle.DOWN,0.1),
                        RobotActions.setRollers(1, rollerIntakeSeconds)))
                .lineToXConstantHeading(bumpSpecimen)
                .afterTime(0.0, new SequentialAction(
                        RobotActions.transferToClaw(),
                        RobotActions.setupChamberFromBack()
                ))
                .waitSeconds(setupChamberFromBackWait)
        // Moving to chamber
                .splineToLinearHeading(new Pose2d(xSubmersibleSpecimen, ySubmersibleSpecimen, Math.toRadians(-90)),Math.toRadians(180))
                // Scoring 1st Specimen
                .stopAndAdd(new SequentialAction(
                        RobotActions.setWrist(Arm.WristAngle.CHAMBER_BACK_AUTON, 0.1),
                        new SleepAction(transferSpecimenToClawWait),
                        RobotActions.setExtendo(Extendo.Extension.EXTENDED, 0)
                ))
//                // Moving to intake 2nd Specimen
                .setTangent(-45)
                .afterTime(unclampAfterTimeWait, new SequentialAction(
                        RobotActions.setArm(Arm.ArmAngle.NEUTRAL, UnclampSpecimenWait),
                        RobotActions.setClaw(Claw.ClawAngles.DEPOSIT_ANGLE, 0),
                        RobotActions.retractToNeutral(0)
                ))




                .lineToYLinearHeading(-45,Math.toRadians(-45))
                .stopAndAdd( new SequentialAction(
                        RobotActions.setV4B(Intake.V4BAngle.DOWN,0.1),
                        RobotActions.setRollers(1, rollerIntakeSeconds)))
                .lineToX(bumpSpecimen)
                .afterTime(0.0, new SequentialAction(
                        RobotActions.transferToClaw(),
                        RobotActions.setupChamberFromBack()
                ))
                // Scoring 2nd Specimen
                .waitSeconds(setupChamberFromBackWait)
                .splineToLinearHeading(new Pose2d(xSubmersibleSpecimen - 1, ySubmersibleSpecimen, Math.toRadians(-90)),Math.toRadians(180))
                .stopAndAdd(new SequentialAction(
                        RobotActions.setWrist(Arm.WristAngle.CHAMBER_BACK_AUTON, 0.1),
                        new SleepAction(transferSpecimenToClawWait),
                        RobotActions.setExtendo(Extendo.Extension.EXTENDED, 0)
                ))
                .setTangent(-45)
                .afterTime(unclampAfterTimeWait, new SequentialAction(
                        RobotActions.setArm(Arm.ArmAngle.NEUTRAL, UnclampSpecimenWait),
                        RobotActions.setClaw(Claw.ClawAngles.DEPOSIT_ANGLE, 0),
                        RobotActions.retractToNeutral(0)
                ))
                .lineToYLinearHeading(-45,Math.toRadians(-45))

                .stopAndAdd( new SequentialAction(
                        RobotActions.setV4B(Intake.V4BAngle.DOWN,0.1),
                        RobotActions.setRollers(1, rollerIntakeSeconds)))
                .lineToX(bumpSpecimen)
                .afterTime(0.0, new SequentialAction(
                        RobotActions.transferToClaw(),
                        RobotActions.setupChamberFromBack()
                ))
                // Scoring 2nd Specimen
                .waitSeconds(setupChamberFromBackWait)
                .splineToLinearHeading(new Pose2d(xSubmersibleSpecimen - 2, ySubmersibleSpecimen, Math.toRadians(-90)),Math.toRadians(180))
                .stopAndAdd(new SequentialAction(
                        RobotActions.setWrist(Arm.WristAngle.CHAMBER_BACK_AUTON, 0.1),
                        new SleepAction(transferSpecimenToClawWait),
                        RobotActions.setExtendo(Extendo.Extension.EXTENDED, 0)
                ))
                .setTangent(-45)
                .afterTime(unclampAfterTimeWait, new SequentialAction(
                        RobotActions.setArm(Arm.ArmAngle.NEUTRAL, UnclampSpecimenWait),
                        RobotActions.setClaw(Claw.ClawAngles.DEPOSIT_ANGLE, 0),
                        RobotActions.retractToNeutral(0)
                ))
                .lineToYLinearHeading(-45,Math.toRadians(-45))

//                .stopAndAdd( new SequentialAction(
//                        RobotActions.setV4B(Intake.V4BAngle.DOWN,0.1),
//                        RobotActions.setRollers(1, rollerIntakeSeconds)))
        ;

        // Moving to intake 3rd Specimen
//                .afterTime(0.0, RobotActions.extendIntake(Extendo.Extension.ONE_FOURTH))
//                .lineToYLinearHeading(-45,Math.toRadians(-40))
//                .stopAndAdd(RobotActions.setRollers(1, 0.5))
//                .afterTime(0.0, new SequentialAction(
//                        RobotActions.transferToClaw(),
//                        RobotActions.setupChamberFromBack()
//                ))
//                // Scoring 3rd Specimen
//                .lineToYLinearHeading(-36,Math.toRadians(270))
//                .stopAndAdd( RobotActions.scoreChamberFromFrontAndRetract());
        return builder;
    }

    private TrajectoryActionBuilder giveSamples(TrajectoryActionBuilder builder) {
        builder = builder
                //moving to first sample
                .setTangent(Math.toRadians(270))
                .afterTime(0, RobotActions.setV4B(Intake.V4BAngle.HOVERING, 0))
                .splineToConstantHeading(new Vector2d(26,-38),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(xSample1,ySample1,Math.toRadians(firstSampleAngle)),Math.toRadians(35))
                // Intake 1st sample
                .stopAndAdd(new SequentialAction(
                        RobotActions.setRollers(1, 0.1),
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0.1)
                ))
                .lineToY(bumpSample)
                .afterTime(0, RobotActions.setV4B(Intake.V4BAngle.UP, 0.1))
                // give sampole1
                .splineToLinearHeading(new Pose2d((xSample2 + xSample1) / 2, ySample2, Math.toRadians(300)), Math.toRadians(300))
                .stopAndAdd( new SequentialAction(
                        RobotActions.setExtendo(Extendo.Extension.EXTENDED, 0),
                        RobotActions.setRollers(-0.8,0.3),
                        RobotActions.setRollers(0, 0),
                        RobotActions.setExtendo(Extendo.LINKAGE_ONE_FOURTH_ANGLE, retractExtendoWait)

                ))
                // Sample 2 intake
                .splineToLinearHeading(new Pose2d(xSample2,ySample2,Math.toRadians(25)),Math.toRadians(35))

                .stopAndAdd(new SequentialAction(
                        RobotActions.setExtendo(Extendo.Extension.ONE_FOURTH, 0),
                        RobotActions.setRollers(1, 0.1),
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0.1)
                ))
                .lineToY(bumpSample)
                .afterTime(0, RobotActions.setV4B(Intake.V4BAngle.UP, 0.2))
                .splineToLinearHeading(new Pose2d((xSample2 + xSample1) / 2, ySample2, Math.toRadians(300)), Math.toRadians(300))
                .stopAndAdd(new SequentialAction(
                        RobotActions.setExtendo(Extendo.Extension.EXTENDED, 0.1),
                        RobotActions.setRollers(-0.8,0.3),
                        RobotActions.setRollers(0, 0.1),
                        RobotActions.setExtendo(Extendo.LINKAGE_ONE_FOURTH_ANGLE, retractExtendoWait)

                ))

                .splineToLinearHeading(new Pose2d(xSample3, ySample3, Math.toRadians(intakeSampleAngle3)), Math.toRadians(10))
                .stopAndAdd(new SequentialAction(
                        RobotActions.setExtendo(extendoAngleSample3, 0),
                        RobotActions.setRollers(1, .2),
                        RobotActions.setV4B(Intake.V4BAngle.DOWN, 0.2)
                ))
                .lineToY(bumpSample3)
                // Intake sample 3
                .afterTime(0, new ParallelAction(
                        RobotActions.setV4B(Intake.V4BAngle.UP, 0.2),
                        RobotActions.setExtendo(Extendo.LINKAGE_MIN_ANGLE, retractExtendoWait)

                ))
                .afterTime(.5, new SequentialAction(
                        RobotActions.setExtendo(Extendo.Extension.EXTENDED, 0.3),
                        RobotActions.setRollers(-0.8,0.3),
                        RobotActions.setRollers(0, 0.1)
//                        RobotActions.setV4B(Intake.V4BAngle.DOWN,0.1)
//                        RobotActions.setExtendo(Extendo.LINKAGE_MIN_ANGLE, retractExtendoWait)

                ))
                .splineToLinearHeading(new Pose2d((xSample2 + xSample1) / 2, ySample2, Math.toRadians(300)), Math.toRadians(300))
                ;
                // Given Sample 3;
        return builder;
    }

    private TrajectoryActionBuilder scoreSpecimen(TrajectoryActionBuilder builder) {
        builder = builder
                .afterTime(0.0, new ParallelAction(
                        RobotActions.setupChamberFromFront(),
                        RobotActions.setClaw(Claw.ClawAngles.CLAMP_ANGLE, 0.0)
                ))
                .lineToY(scoreSpecimenY)
                .stopAndAdd( RobotActions.scoreChamberFromFrontAndRetract());
        return builder;
    }
}
