package org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_435;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.robot.intothedeep.subsystem.Common.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.FeedforwardGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;

@Config
public final class Lift {

    /**
     * A PIDGains object being set to certain values (tweak these numbers!!)
     */
    public static PIDGains pidGains = new PIDGains(
            0.005,
            0.002,
            0.0001,
            Double.POSITIVE_INFINITY
    );

    public static FeedforwardGains feedforwardGains = new FeedforwardGains(
            0.01,
            0.01
    );

    // Sets the filter for PID outputs and constrains overshoots with controlling (also tweak!!)
    public static LowPassGains filterGains = new LowPassGains(0, 2);

    /**
     * Sets the constants for the positions, conversions, etc
     * Remember to set these constants correctly! (in ticks)
     */
    public static int
            MAX_MOTOR_TICKS = 2550,
            MIN_MOTOR_TICKS = -5,
            LOW_BASKET_TICKS = 297,
            HIGH_BASKET_TICKS = 2050,
            BACK_WALL_PICKUP_TICKS = 953,
            BEFORE_BACK_SPECIMEN_TICKS = 1043,
            BACK_WALL_SPECIMEN_SETUP_TICKS = 1628,
            LEVEL_TWO_CLIMB_SETUP_TICKS = 2550,
            LEVEL_TWO_CLIMB_TICKS = 1716,
            LEVEL_THREE_CLIMB_SETUP_TICKS = 2550,
            LEVEL_THREE_CLIMB_TICKS = 2297,
            UNSAFE_THRESHOLD_TICKS = 829,
            RETRACTED_THRESHOLD_TICKS = 41,
            CHAMBER_HANG_OVERRIDE_TICKS = 50;

    public static double
            kG = 0.011065,
            RESETTING_POWER = -0.02,
            JOYSTICK_MULTIPLIER = 40; // 1 = 40 ticks

    private final MotorEx[] motors;

    private final Motor.Encoder encoder;

    private final FIRLowPassFilter derivFilter = new FIRLowPassFilter(filterGains);
    private final PIDController controller = new PIDController(derivFilter);

    private final VoltageSensor batteryVoltageSensor;

    private double position = 0;

    public enum Ticks {
        RETRACTED,
        LOW_BASKET,
        HIGH_BASKET,
        BACK_WALL_PICKUP,
        BEFORE_BACK_SPECIMEN,
        SETUP_BACK_WALL_SPECIMEN,
        LEVEL_TWO_CLIMB_SETUP,
        LEVEL_TWO_CLIMB,
        LEVEL_THREE_CLIMB_SETUP,
        LEVEL_THREE_CLIMB,
        CHAMBER_HANG_OVERRIDE,
            EXTENDED;

        private int toTicks() {
            switch (this) {
                case LOW_BASKET:                return LOW_BASKET_TICKS;
                case HIGH_BASKET:               return HIGH_BASKET_TICKS;
                case BACK_WALL_PICKUP:          return BACK_WALL_PICKUP_TICKS;
                case BEFORE_BACK_SPECIMEN:      return BEFORE_BACK_SPECIMEN_TICKS;
                case SETUP_BACK_WALL_SPECIMEN:  return BACK_WALL_SPECIMEN_SETUP_TICKS;
                case LEVEL_TWO_CLIMB_SETUP:     return LEVEL_TWO_CLIMB_SETUP_TICKS;
                case LEVEL_TWO_CLIMB:           return LEVEL_TWO_CLIMB_TICKS;
                case LEVEL_THREE_CLIMB_SETUP:   return LEVEL_THREE_CLIMB_SETUP_TICKS;
                case LEVEL_THREE_CLIMB:         return LEVEL_THREE_CLIMB_TICKS;
                case EXTENDED:                  return MAX_MOTOR_TICKS;
                case CHAMBER_HANG_OVERRIDE:     return CHAMBER_HANG_OVERRIDE_TICKS;
                case RETRACTED: default:        return MIN_MOTOR_TICKS;
            }
        }

        public boolean isArmUnsafe() {
            return toTicks() <= UNSAFE_THRESHOLD_TICKS;
        }
    }

    private Ticks targetTicks = Ticks.RETRACTED;

    public boolean isLocked = false;

    private double manualPower;

    /**
     * Constructor of Lift class; Sets variables with hw (hardwareMap)
     * @param hardwareMap; A constant map that holds all the parts for config in code
     */
    public Lift(HardwareMap hardwareMap) {
        MotorEx leader = new MotorEx(hardwareMap, "leader", RPM_435);
        MotorEx follower = new MotorEx(hardwareMap, "follower", RPM_435);
        MotorEx follower2 = new MotorEx(hardwareMap, "follower2", RPM_435);

        encoder = follower2.encoder;
        encoder.reset();

        follower.setInverted(true);
        follower2.setInverted(true);

        encoder.setDirection(Motor.Direction.REVERSE);

        motors = new MotorEx[] {leader, follower, follower2};

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public Ticks getTargetTicks() {
        return targetTicks;
    }

    public boolean setTargetTicks(Ticks ticks, boolean isOverride ) {
        if (isLocked && !isOverride) return false;
        targetTicks = ticks;
        controller.setTarget(new State(getTargetTicks().toTicks()));

        return true;
    }

    public void runManual(double power) {
        manualPower = power;
    }

    public void reset() {
        encoder.reset();
        controller.reset();

        setTargetTicks(Ticks.RETRACTED, true);
        position = 0;
    }

    public boolean setTargetTicks(Ticks ticks) {
        return setTargetTicks(ticks, false);
    }

    /**
     * Sets the three variables; The currentState sets to the position of the motor; Other two set the constants of the gains
     * Calls another run() method that calculates the motor output proportionally and doesn't compensate for power
     */
    public void run() {
        position = encoder.getPosition();

        double scalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        double output = position >= RETRACTED_THRESHOLD_TICKS ? kG * scalar : 0;


         if (manualPower != 0) {
            controller.setTarget(new State(position));
            output += manualPower;

        } else if (targetTicks == Ticks.RETRACTED && position <= RETRACTED_THRESHOLD_TICKS) {
            output = RESETTING_POWER;
            this.reset();

        } else {

            controller.setGains(pidGains);
            derivFilter.setGains(filterGains);

            output += controller.calculate(new State(position));

        }

        for (MotorEx motor : motors) motor.set(output);
    }

    public void printTelemetry() {
        mTelemetry.addData("Target position (ticks)", getTargetTicks().toTicks());
        mTelemetry.addData("Current position (ticks)", position);
        mTelemetry.addData("Current state (name)", getTargetTicks().name());
        mTelemetry.addData("Manual power (%)", manualPower * 100);
    }
}
