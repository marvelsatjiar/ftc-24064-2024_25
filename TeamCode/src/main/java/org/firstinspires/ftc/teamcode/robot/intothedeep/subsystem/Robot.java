package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.util.SimpleServoPivot.getGoBildaServo;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.BulkReader;
import org.firstinspires.ftc.teamcode.util.LoopUtil;
import org.firstinspires.ftc.teamcode.util.SimpleServoPivot;

@Config
public final class Robot {
    public final MecanumDrive drivetrain;
    public final Extendo extendo;
    public final Intake intake;
    public final BulkReader bulkReader;
    public final Claw claw;
    public final Lift lift;
    public final Arm arm;
    public final ActionScheduler actionScheduler;
    public final Sweeper sweeper;
//    public final AutoAligner autoAligner;

    public final double
        SWING_CORRECTOR_INACTIVE = 0,
        SWING_CORRECTOR_ACTIVE = 90;

    public final SimpleServoPivot swingCorrector;

    public enum State {
        NEUTRAL,
        EXTENDO_OUT,
        FRONT_WALL_PICKUP,
        SETUP_FRONT_SPECIMEN_FROM_WALL,
        TO_BE_TRANSFERRED,
        TRANSFERRED,
        SETUP_CHAMBER_FROM_BACK,
        SETUP_CHAMBER_FROM_FRONT,
        SETUP_SCORE_BASKET,
        SETUP_LEVEL_TWO_HANG,
        CLIMB_LEVEL_TWO_HANG,
        SETUP_LEVEL_THREE_HANG,
        CLIMB_LEVEL_THREE_HANG,
        SCORED_SAMPLE_HIGH_BASKET,
        SETUP_DROP_SAMPLE,
    }

    State currentState = State.NEUTRAL;

    /**
     * Constructor used in teleOp classes that makes the current pose2d, 0
     * @param hardwareMap A constant map that holds all the parts for config in code
     */
    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
    }

    /**
     * Constructor for instantiating a new 'robot' class
     * @param hardwareMap: A constant map that holds all the parts for config in code
     * @param pose2d: The current pose for the robot, which is currently zero at start of teleOp
     */
    public Robot(HardwareMap hardwareMap, Pose2d pose2d) {
        swingCorrector = new SimpleServoPivot(SWING_CORRECTOR_INACTIVE, SWING_CORRECTOR_ACTIVE, getGoBildaServo(hardwareMap, "swing corrector"));

        drivetrain = new MecanumDrive(hardwareMap, pose2d);
        extendo = new Extendo(hardwareMap);
        bulkReader = new BulkReader(hardwareMap);
        intake = new Intake(hardwareMap);
        claw = new Claw(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        sweeper = new Sweeper(hardwareMap);
//        autoAligner = new AutoAligner(hardwareMap);
        actionScheduler = new ActionScheduler();
    }

    // Reads all the necessary sensors (including battery volt.) in one bulk read
    public void readSensors() {
        bulkReader.bulkRead();
    }

    // Runs all the necessary mechanisms
    public void run() {

        actionScheduler.run();
        extendo.run(intake.getTargetV4BAngle().isV4BUnsafe());
        sweeper.run();
        intake.run();
        lift.run();
        claw.run();
        arm.run(lift.getTargetTicks().isArmUnsafe());
    }

    // Prints data on the driver hub for debugging and other uses
    public void printTelemetry() {
        mTelemetry.addData("Robot State", robot.currentState.name());
        mTelemetry.addData("Loop time (hertz)", LoopUtil.getLoopTimeInHertz());
//        extendo.printTelemetry();
//        lift.printTelemetry();
//        arm.printTelemetry();
        intake.printTelemetry();
//        autoAligner.printTelemetry();
        mTelemetry.update();
    }

    public Robot.State getCurrentState() {
        return currentState;
    }
}
