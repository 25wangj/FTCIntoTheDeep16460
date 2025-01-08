package org.firstinspires.ftc.teamcode.command;
import java.util.Arrays;
import java.util.function.DoubleConsumer;
public class WaitCommand extends Command {
    private DoubleConsumer initFn;
    private DoubleConsumer endFn;
    private double end;
    private double seconds;
    public WaitCommand(double seconds, Subsystem... systems) {
        this(t -> {}, seconds, false, systems);
    }
    public WaitCommand(double seconds, boolean cancelable, Subsystem... systems) {
        this(t -> {}, seconds, cancelable, systems);
    }
    public WaitCommand(DoubleConsumer initFn, double seconds, Subsystem... systems) {
        this(initFn, seconds, false, systems);
    }
    public WaitCommand(DoubleConsumer initFn, double seconds, boolean cancelable, Subsystem... systems) {
        this(initFn, seconds, t -> {}, cancelable, systems);
    }
    public WaitCommand(double seconds, DoubleConsumer endFn, Subsystem... systems) {
        this(seconds, endFn, false, systems);
    }
    public WaitCommand(double seconds, DoubleConsumer endFn, boolean cancelable, Subsystem... systems) {
        this(t -> {}, seconds, endFn, cancelable, systems);
    }
    public WaitCommand(DoubleConsumer initFn, double seconds, DoubleConsumer endFn, Subsystem... systems) {
        this(initFn, seconds, endFn, false, systems);
    }
    public WaitCommand(DoubleConsumer initFn, double seconds, DoubleConsumer endFn, boolean cancelable, Subsystem... systems) {
        subsystems.addAll(Arrays.asList(systems));
        this.initFn = initFn;
        this.endFn = endFn;
        this.cancelable = cancelable;
        this.seconds = seconds;
    }
    @Override
    public void init(double time) {
        end = time + seconds;
        initFn.accept(time);
    }
    @Override
    public void run(double time) {}
    @Override
    public void end(double time, boolean canceled) {
        if (!canceled) {
            endFn.accept(time);
        }
    }
    @Override
    public boolean done(double time) {
        return time > end;
    }
}