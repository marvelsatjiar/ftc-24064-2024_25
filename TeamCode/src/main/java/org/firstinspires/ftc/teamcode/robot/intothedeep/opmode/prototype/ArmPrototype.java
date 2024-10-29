package org.firstinspires.ftc.teamcode.robot.intothedeep.opmode.prototype;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Arm;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Lift;

@Config
@TeleOp(name = "Arm Prototype",  group = "Prototype")
public final class ArmPrototype extends LinearOpMode {
    Lift lift;
    Arm arm;
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            Arm.Position targetPosition = Arm.Position.COLLECTING;
            Lift.SlideTicks targetTicks = Lift.SlideTicks.RETRACTED;

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
                    if (isDPADUpPressed) targetTicks = Lift.SlideTicks.BASKET;
                    break;
                case BASKET:
                    if (isDPADUpPressed) targetTicks = Lift.SlideTicks.CHAMBER;
                    if (isDPADDownPressed) targetTicks = Lift.SlideTicks.RETRACTED;
                    break;
                case CHAMBER:
                    if (isDPADUpPressed) targetTicks = Lift.SlideTicks.CLIMB;
                    if (isDPADDownPressed) targetTicks = Lift.SlideTicks.BASKET;
                    break;
                case CLIMB:
                    if (isDPADUpPressed) targetTicks = Lift.SlideTicks.EXTENDED;
                    if (isDPADDownPressed) targetTicks = Lift.SlideTicks.CHAMBER;
                    break;
                case EXTENDED:
                    if (isDPADDownPressed) targetTicks = Lift.SlideTicks.CLIMB;
                    break;
            }
            arm.setTarget(targetPosition);
            lift.setPosition(targetTicks);

            arm.run(lift.getSetPoint().isArmUnsafe());
            lift.run();
        }
    }
}
