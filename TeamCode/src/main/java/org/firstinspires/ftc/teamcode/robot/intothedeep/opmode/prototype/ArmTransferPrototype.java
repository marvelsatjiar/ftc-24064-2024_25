package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode.prototype;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;

import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Lift;

@Config
@TeleOp(name = "Arm Transfer Prototype",  group = "Prototype")
public final class ArmTransferPrototype extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        Lift lift = new Lift(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        Arm.Position targetPosition = Arm.Position.COLLECTING;
        Lift.Ticks targetTicks = Lift.Ticks.RETRACTED;

        mTelemetry = new MultipleTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            boolean isAPressed = gamepadEx1.wasJustPressed(GamepadKeys.Button.A);
            boolean isDPADUpPressed = gamepadEx1.wasJustPressed(DPAD_UP);
            boolean isDPADDownPressed = gamepadEx1.wasJustPressed(DPAD_DOWN);

            switch (targetPosition) {
                case COLLECTING:
                    if (isAPressed) targetPosition = Arm.Position.HIGH_BASKET;
                    break;
                case HIGH_BASKET:
                    if (isAPressed) targetPosition = Arm.Position.LOW_BASKET;
                    break;
                case LOW_BASKET:
                    if (isAPressed) targetPosition = Arm.Position.HIGH_CHAMBER_UPWARDS;
                    break;
                case HIGH_CHAMBER_UPWARDS:
                    if (isAPressed) targetPosition = Arm.Position.HIGH_CHAMBER_DOWNWARDS;
                    break;
                case HIGH_CHAMBER_DOWNWARDS:
                    if (isAPressed) targetPosition = Arm.Position.COLLECTING;
                    break;
            }

            switch (targetTicks) {
                case RETRACTED:
                    if (isDPADUpPressed) targetTicks = Lift.Ticks.HIGH_BASKET;
                    break;
                case HIGH_BASKET:
                    if (isDPADUpPressed) targetTicks = Lift.Ticks.LOW_CHAMBER;
                    if (isDPADDownPressed) targetTicks = Lift.Ticks.RETRACTED;
                    break;
                case LOW_CHAMBER:
                    if (isDPADUpPressed) targetTicks = Lift.Ticks.CLIMB;
                    if (isDPADDownPressed) targetTicks = Lift.Ticks.HIGH_BASKET;
                    break;
                case CLIMB:
                    if (isDPADUpPressed) targetTicks = Lift.Ticks.EXTENDED;
                    if (isDPADDownPressed) targetTicks = Lift.Ticks.LOW_CHAMBER;
                    break;
                case EXTENDED:
                    if (isDPADDownPressed) targetTicks = Lift.Ticks.CLIMB;
                    break;
            }
            arm.setTargetPosition(targetPosition);
            lift.setTargetTicks(targetTicks);

            arm.run(lift.getTargetTicks().isArmUnsafe());
            lift.run();

            lift.printTelemetry();
            arm.printTelemetry();
            mTelemetry.update();
        }
    }
}
