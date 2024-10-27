package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode.prototype;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;

@Config
@TeleOp(name = "Intake Prototype",  group = "Prototype")
public final class IntakePrototype extends LinearOpMode {
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        intake = new Intake(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(DPAD_UP)) intake.setTarget(Intake.V4BAngles.UP);
            if (gamepadEx1.wasJustPressed(DPAD_DOWN)) intake.setTarget(Intake.V4BAngles.DOWN);
            if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) intake.setTarget(Intake.V4BAngles.CLEARING);

            double trigger1 = gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER);

            intake.setServoPower(trigger1);

        }
    }
}
