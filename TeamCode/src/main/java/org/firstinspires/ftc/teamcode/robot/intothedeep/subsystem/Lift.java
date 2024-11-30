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
            MAX_MOTOR_TICKS = 2970,
            MIN_MOTOR_TICKS = -5,
            LOW_BASKET_TICKS = 600,
            HIGH_BASKET_TICKS = 2970,
            HIGH_CHAMBER_SCORE_BACK_TICKS = 1300,
            HIGH_CHAMBER_SCORE_FRONT_TICKS = 1650,
            HIGH_CHAMBER_SETUP_BACK_TICKS = 485,
            HIGH_CHAMBER_SETUP_FRONT_TICKS = 1100,
            WALL_PICKUP_TICKS = 1150,
            INTERMEDIARY_WALL_PICKUP_TICKS = 1300,
            CLIMB_TICKS = 1200,
            UNSAFE_THRESHOLD_TICKS = 1000;

    public static double
            kG = 0.011065,
            JOYSTICK_MULTIPLIER = 40; // 1 = 40 ticks

    private final MotorEx[] motors;

    private final Motor.Encoder encoder;

    private final FIRLowPassFilter derivFilter = new FIRLowPassFilter(filterGains);
    private final PIDController controller = new PIDController(derivFilter);

    private final VoltageSensor batteryVoltageSensor;

    private State currentState = new State();

    public enum Ticks {
        RETRACTED,
        LOW_BASKET,
        HIGH_BASKET,
        HIGH_CHAMBER_SCORE_BACK,
        HIGH_CHAMBER_SCORE_FRONT,
        HIGH_CHAMBER_SETUP_BACK,
        HIGH_CHAMBER_SETUP_FRONT,
        WALL_PICKUP,
        CLIMB,
        INTERMEDIARY_WALL_PICKUP,
        EXTENDED;

        private int toTicks() {
            switch (this) {
                case LOW_BASKET:                return LOW_BASKET_TICKS;
                case HIGH_BASKET:               return HIGH_BASKET_TICKS;
                case HIGH_CHAMBER_SCORE_BACK:   return HIGH_CHAMBER_SCORE_BACK_TICKS;
                case HIGH_CHAMBER_SCORE_FRONT:  return HIGH_CHAMBER_SCORE_FRONT_TICKS;
                case HIGH_CHAMBER_SETUP_BACK:   return HIGH_CHAMBER_SETUP_BACK_TICKS;
                case HIGH_CHAMBER_SETUP_FRONT:  return HIGH_CHAMBER_SETUP_FRONT_TICKS;
                case WALL_PICKUP:               return WALL_PICKUP_TICKS;
                case INTERMEDIARY_WALL_PICKUP:  return INTERMEDIARY_WALL_PICKUP_TICKS;
                case CLIMB:                     return CLIMB_TICKS;
                case EXTENDED:                  return MAX_MOTOR_TICKS;
                case RETRACTED: default:        return MIN_MOTOR_TICKS;
            }
        }

        public boolean isArmUnsafe() {
            return toTicks() <= UNSAFE_THRESHOLD_TICKS;
        }
    }

    private Ticks targetTicks = Ticks.RETRACTED;

    public boolean isLocked = false;

    /**
     * Constructor of Lift class; Sets variables with hw (hardwareMap)
     * @param hardwareMap; A constant map that holds all the parts for config in code
     */
    public Lift(HardwareMap hardwareMap) {
        MotorEx leader = new MotorEx(hardwareMap, "leader", RPM_435);
        MotorEx follower = new MotorEx(hardwareMap, "follower", RPM_435);

        encoder = new MotorEx(hardwareMap, "dummy motor", RPM_435).encoder;
        encoder.reset();

        follower.setInverted(true);

        motors = new MotorEx[] {leader, follower};

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

    public boolean setTargetTicks(Ticks ticks) {
        return setTargetTicks(ticks, false);
    }

    /**
     * Sets the three variables; The currentState sets to the position of the motor; Other two set the constants of the gains
     * Calls another run() method that calculates the motor output proportionally and doesn't compensate for power
     */
    public void run() {
        currentState = new State(encoder.getPosition());
        controller.setGains(pidGains);
        derivFilter.setGains(filterGains);

        run(controller.calculate(currentState), false);
    }

    public void run(double motorPower) {
        run(motorPower, true);
    }

    /**
     * Checks if the voltage is to be saved, if true, use scalar to modify motor power
     * @param motorPower; The power that is set to be used to set the motor's power
     * @param voltageCompensate; Boolean that is used if battery is low, and if it needs to compensate (save)
     */
    public void run(double motorPower, boolean voltageCompensate) {
        double scalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();

        if (voltageCompensate) motorPower *= scalar;

        for (MotorEx motor : motors) motor.set(motorPower + kG * scalar);
    }

    public void printTelemetry() {
        mTelemetry.addData("Actual position (ticks)", encoder.getPosition());
        mTelemetry.addData("Target position (ticks)", getTargetTicks().toTicks());
        mTelemetry.addData("Current state (name)", getTargetTicks().name());
    }
}
