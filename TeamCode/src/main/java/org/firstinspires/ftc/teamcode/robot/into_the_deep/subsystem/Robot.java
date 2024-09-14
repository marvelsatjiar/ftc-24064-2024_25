package org.firstinspires.ftc.teamcode.robot.into_the_deep.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.BulkReader;

@Config
public class Robot {
    // Constants & classes needed to run for this robot, initialized and/or associated w/ a value
    public final static double MAX_VOLTAGE = 13;

    public final MecanumDrive drivetrain;
    public final Extendo extendo;
    public final BulkReader bulkReader;
    public MotorEx intake;

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
        drivetrain = new MecanumDrive(hardwareMap, pose2d);
        extendo = new Extendo(hardwareMap);
        bulkReader = new BulkReader(hardwareMap);

        intake = new MotorEx(hardwareMap, "intake");
    }

    // Reads all the necessary sensors (including battery volt.) in one bulk read
    public void readSensors() {bulkReader.bulkRead();}

    // Runs all the necessary mechanisms
    public void run() {
        extendo.run();
    }

    // Prints data on the driver hub for debugging and other uses
    public void printTelemetry() {
        extendo.printTelemetry();
    }
}
