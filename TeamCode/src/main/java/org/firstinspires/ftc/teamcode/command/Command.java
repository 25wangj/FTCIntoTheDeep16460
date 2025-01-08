package org.firstinspires.ftc.teamcode.command;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
public abstract class Command {
    protected Set<Subsystem> subsystems = new HashSet<>();
    protected boolean cancelable = false;
    public Command(Subsystem... systems) {
        subsystems.addAll(Arrays.asList(systems));
    }
    public abstract void init(double time);
    public abstract void run(double time);
    public abstract void end(double time, boolean canceled);
    public Set<Subsystem> getSubsystems() {
        return subsystems;
    }
    public boolean isCancelable() {
        return cancelable;
    }
    public abstract boolean done(double time);
}
