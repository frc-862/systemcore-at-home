package frc.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;

/**
 * Documents how cleanup works when a v3 command is canceled. A canceled coroutine is simply never
 * resumed, so a finally block after park() does not run. Use whenCanceled() for StartEnd-style
 * cleanup.
 */
class CoroutineCancelTest {
    @Test
    void finallyDoesNotRunButWhenCanceledDoes() {
        Scheduler scheduler = Scheduler.createIndependentScheduler();
        AtomicBoolean started = new AtomicBoolean(false);
        AtomicBoolean finallyRan = new AtomicBoolean(false);
        AtomicBoolean whenCanceledRan = new AtomicBoolean(false);

        Command command = Command.noRequirements(co -> {
            try {
                started.set(true);
                co.park();
            } finally {
                finallyRan.set(true);
            }
        }).whenCanceled(() -> whenCanceledRan.set(true)).named("Park");

        scheduler.schedule(command);
        scheduler.run();
        assertTrue(started.get());

        scheduler.cancel(command);
        scheduler.run();

        assertFalse(finallyRan.get());
        assertTrue(whenCanceledRan.get());
    }
}
