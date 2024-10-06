package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.BulkReader;

@Config
public class ITDRobot {
    // Constants & classes needed to run for this robot, initialized and/or associated w/ a value
    public final static double MAX_VOLTAGE = 13;

    public final MecanumDrive drivetrain;
    public final Extendo extendo;
    public final Intake intake;
    public final BulkReader bulkReader;
    public final Claw claw;

    /**
     * Constructor used in teleOp classes that makes the current pose2d, 0
     * @param hardwareMap A constant map that holds all the parts for config in code
     */
    public ITDRobot(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
    }

    /**
     * Constructor for instantiating a new 'robot' class
     * @param hardwareMap: A constant map that holds all the parts for config in code
     * @param pose2d: The current pose for the robot, which is currently zero at start of teleOp
     */
    public ITDRobot(HardwareMap hardwareMap, Pose2d pose2d) {
        drivetrain = new MecanumDrive(hardwareMap, pose2d);
        extendo = new Extendo(hardwareMap);
        bulkReader = new BulkReader(hardwareMap);
        intake = new Intake(hardwareMap);
        claw = new Claw(hardwareMap);
    }

    // Reads all the necessary sensors (including battery volt.) in one bulk read
    public void readSensors() {bulkReader.bulkRead();}

    // Runs all the necessary mechanisms
    public void run() {
        extendo.run();
        intake.run();
        claw.run();
    }

    // Prints data on the driver hub for debugging and other uses
    public void printTelemetry() {
        extendo.printTelemetry();
    }
}
