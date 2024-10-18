package org.firstinspires.ftc.teamcode.sensor.vision;
// Package declaration for organizing the class

import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainTeleOp.mTelemetry;
// Imports mTelemetry from another class for use in telemetry updates

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
// Imports necessary Limelight classes for vision system stuff

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// Imports FTC Java classes for opmode

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
// Imports Pose3D class to handle 3D robot position data

import java.util.List;
// Imports Java's List class for handling collections of results


@TeleOp(name = "Limelight", group = "Sensor")
// Declares this class as a teleoperated opmode
@Disabled
// Prevents this opmode from being displayed on the driver station until enabled
public class Limelight extends LinearOpMode {
// Declares the Limelight class, extending LinearOpMode for FTC operation

    private Limelight3A limelight;
    // Declares a variable to hold the Limelight hardware object

    @Override
    public void runOpMode() {
        // Overrides the runOpMode method, which will be executed when the opmode starts
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // Initializes the limelight hardware by mapping it to the robot configuration

        mTelemetry.setMsTransmissionInterval(11);
        // Sets the telemetry update interval to 11 milliseconds

        limelight.pipelineSwitch(0);
        // Switches the Limelight vision system to pipeline 0

        limelight.start();
        // Starts the Limelight, enabling data collection

        mTelemetry.addData(">", "Robot Ready.  Press Play.");
        // Adds a message to the telemetry indicating that the robot is ready
        mTelemetry.update();
        // Updates the telemetry to display the message
        waitForStart();
        // Waits for the user to press "Play" to start the opmode

        while (opModeIsActive()) {
            // The main loop that continues running as long as the opmode is active

            LLStatus status = limelight.getStatus();
            // Retrieves the current status of the Limelight
            mTelemetry.addData("Name", "%s", status.getName());
            //%s is string
            // Adds the Limelight's name to telemetry
            mTelemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(), (int)status.getFps());
            //%1fC is a float that will give to one decimal place Celsius
            //%% will give actual percentage
            //%d is integer
            // Adds the temperature, CPU usage, and frame rate of the Limelight to telemetry
            mTelemetry.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());
            // Adds the current pipeline index and type to telemetry

            LLResult result = limelight.getLatestResult();
            // Retrieves the latest result from the Limelight
            if (result != null) {
                // Checks if the result is valid (i.e., not null)

                Pose3D botpose = result.getBotpose();
                // Retrieves the 3D pose (position and orientation) of the robot
                double captureLatency = result.getCaptureLatency();
                // Retrieves the latency for capturing images from the Limelight
                double targetingLatency = result.getTargetingLatency();
                // Retrieves the latency for processing targeting data
                double parseLatency = result.getParseLatency();
                // Retrieves the latency for parsing the result data
                mTelemetry.addData("LL Latency", captureLatency + targetingLatency);
                // Adds the total latency (capture + targeting) to telemetry
                mTelemetry.addData("Parse Latency", parseLatency);
                // Adds the parsing latency to telemetry
                mTelemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
                // Displays any output from Python scripts running on the Limelight

                if (result.isValid()) {
                    // Checks if the result contains valid data

                    mTelemetry.addData("tx", result.getTx());
                    // How far left or right the target is (degrees)
                    mTelemetry.addData("txnc", result.getTxNC());
                    // Adds the non-compensated horizontal offset (degrees) to telemetry
                    mTelemetry.addData("ty", result.getTy());
                    // How far up or down the target is (degrees)
                    mTelemetry.addData("tync", result.getTyNC());
                    // Adds the non-compensated vertical offset (degrees) to telemetry
                    mTelemetry.addData("Botpose", botpose.toString());
                    // Adds the robot's current 3D pose to telemetry

                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    // Retrieves fiducial marker results from the Limelight
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        // Loops through each fiducial marker result
                        mTelemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                        // Adds fiducial ID, family, and position to telemetry
                    }

                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    // Retrieves color detection results from the Limelight
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        // Loops through each color detection result
                        mTelemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                        // Adds the detected color's position to telemetry
                    }
                }
            } else {
                // If no result is available (result is null)
                mTelemetry.addData("Limelight", "No data available");
                // Adds a message indicating no data is available
            }

            mTelemetry.update();
            // Updates telemetry with all the data collected in the loop
        }
        limelight.stop();
        // Stops the Limelight when the opmode ends
    }
}