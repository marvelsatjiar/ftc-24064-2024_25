package org.firstinspires.ftc.teamcode.robot.centerstage.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.centerstage.subsystem.CSRobot;
import org.firstinspires.ftc.teamcode.robot.centerstage.subsystem.Memory;
import org.firstinspires.ftc.teamcode.util.LoopUtil;

@Disabled
public final class MainTeleOp extends LinearOpMode {
    static CSRobot csRobot;
    public static GamepadEx gamepadEx1;
    public static GamepadEx gamepadEx2;

    public static MultipleTelemetry mTelemetry;

    public static boolean keyPressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }

    @Override
    public void runOpMode() {
        boolean isHanging = false;

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        csRobot = new CSRobot(hardwareMap);

        Pose2d endPose = Memory.AUTO_END_POSE;
        if (endPose != null) {
            csRobot.drivetrain.setCurrentHeading(endPose.heading.toDouble() - (Memory.IS_RED ? AbstractAuto.FORWARD : AbstractAuto.BACKWARD));
        }

        csRobot.purplePixel.setActivated(true);

        waitForStart();

        while (opModeIsActive()) {
            // Read sensors + gamepads:
            csRobot.readSensors();
            csRobot.drivetrain.updatePoseEstimate();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            // Gamepad 1
            // Change the heading of the drivetrain in field-centric mode
            double x = gamepadEx1.getRightX();
            if (gamepadEx1.isDown(LEFT_BUMPER)) {
                double y = gamepadEx1.getRightY();
                if (hypot(x, y) >= 0.8) csRobot.drivetrain.setCurrentHeading(atan2(y, x));
                x = 0;
            }

            double slowMult = gamepadEx1.isDown(RIGHT_BUMPER) ? 0.2 : 1;
            csRobot.drivetrain.setFieldCentricPowers(
                    
                    new PoseVelocity2d(
                            new Vector2d(
                                    gamepadEx1.getLeftY() * slowMult,
                                    -gamepadEx1.getLeftX() * slowMult
                            ),
                            -x * slowMult
                    )
            );

//            if (keyPressed(1, B)) robot.deployableRoller.toggle();

            // Gamepad 2

            // Shared
            // The intake power takes precedent to the first player
            double trigger1 = gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER);
            double trigger2 = gamepadEx2.getTrigger(RIGHT_TRIGGER) - gamepadEx2.getTrigger(LEFT_TRIGGER);
            double intake = trigger1 != 0 ? trigger1 : trigger2;
            csRobot.rollers.setIntake(intake);
            csRobot.rollers.setDeployableWithTrigger(intake);

            if (!isHanging) csRobot.run();
            else csRobot.hang(trigger1);

            csRobot.printTelemetry();
            mTelemetry.addData("Loop time (hertz)", LoopUtil.getLoopTimeInHertz());
            mTelemetry.update();
        }
    }
}