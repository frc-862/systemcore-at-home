// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.leds;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.telemetry.Telemetry;

public class LEDMechanism implements Mechanism {
    private final LEDController leds;
    private final boolean[] enabledStates;
    private final LEDBehavior[] ledBehaviors;
    private LEDBehavior defaultBehavior;

    private double loop = 0;

    /**
     * Creates a new LEDMechanism. </p>
     * LEDMechanism manages complex sets of LED behaviors.
     *
     * @param numStates The number of different LED states to manage.
     * @param numLEDs The number of LEDs in the LED controller.
     * @param pwmPort The PWM port the LED controller is connected to.
     */
    public LEDMechanism(int numStates, int numLEDs, int pwmPort) {
        enabledStates = new boolean[numStates];
        ledBehaviors = new LEDBehavior[numStates];
        leds = new LEDController(numLEDs, pwmPort);
        defaultBehavior = new LEDBehavior() { @Override public void apply(LEDController leds) {} };

        Scheduler.getDefault().addPeriodic(this::periodic);
    }

    /**
     * Sets the LED behavior for a specific state ID.
     *
     * @param stateID The ID of the state to set the behavior for.
     * @param behavior The LED behavior to set for the state.
     */
    public void setBehavior(int stateID, LEDBehavior behavior) {
        ledBehaviors[stateID] = behavior;
    }

    /**
     * Sets the default LED behavior.
     *
     * @param behavior The default LED behavior to set.
     */
    public void setDefaultBehavior(LEDBehavior behavior) {
        defaultBehavior = behavior;
    }

    /**
     * Creates a command that enables a specific state. </p>
     * This command will enable the state when started and disable it when ended.
     * It doesn't require the LED mechanism so many states can be enabled at once.
     * v3 commands keep running while disabled, so there is no ignoringDisable().
     *
     * @param stateID The ID of the state to set.
     * @param inverted Shows whether the state is enabled or not.
     * @return A command that enables the specified state.
     */
    public Command enableState(int stateID, boolean inverted) {
        return Command.noRequirements(co -> {
            enabledStates[stateID] = !inverted;
            co.park();
        })
        .whenCanceled(() -> enabledStates[stateID] = inverted)
        .named("LED State " + stateID);
    }

    /**
     * Creates a command that enables a specific state. </p>
     * This command will enable the state when started and disable it when ended.
     *
     * @param stateID The ID of the state to set.
     * @return A command that enables the specified state.
     */
    public Command enableState(int stateID) {
        return enableState(stateID, false);
    }

    public Command disableState(int stateID) {
        return enableState(stateID, true);
    }

    /**
     * Creates a command that enables a specific state for a length of time.
     *
     * @param stateID The ID of the state to set.
     * @param timeoutSeconds The length of time to enable the state for.
     * @return A command that enables the specified state.
     */
    public Command enableStateWithTimeout(int stateID, double timeoutSeconds) {
        // withTimeout() races against the parked command, which cancels it and runs whenCanceled
        return enableState(stateID).withTimeout(Seconds.of(timeoutSeconds));
    }

    /**
     * Sets whether a specific state is enabled.
     *
     * @param stateID The ID of the state to modify.
     * @param enabled true to enable the state, false to disable it.
     */
    public void setState(int stateID, boolean enabled) {
        enabledStates[stateID] = enabled;
    }

    /**
     * Gets if a specific state is enabled.
     *
     * @param stateID The ID of the state to get.
     * @return true if the state is enabled, false otherwise.
     */
    public boolean getState(int stateID) {
        return enabledStates[stateID];
    }

    private void periodic() {
        loop++;
        if (loop % 10 == 0) {
            applyLEDS();
        }

        Telemetry.log("LEDs/EnabledStates", enabledStates);
    }

    private void applyLEDS() {
        defaultBehavior.apply(leds);
        for (int i = enabledStates.length - 1; i >= 0; i--) {
        LEDBehavior behavior = ledBehaviors[i];
        if (enabledStates[i] && behavior != null) {
            behavior.apply(leds);
        }
        }
        leds.apply();
    }
}
