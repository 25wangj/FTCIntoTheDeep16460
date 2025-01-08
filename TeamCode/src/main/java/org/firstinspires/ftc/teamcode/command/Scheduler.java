package org.firstinspires.ftc.teamcode.command;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
public class Scheduler {
    private Set<Command> commands = new HashSet<>();
    private Map<Subsystem, Command> subsystems = new HashMap<>();
    private Set<Listener> listeners = new HashSet<>();
    private Set<Command> added = new HashSet<>();
    private Set<Command> finished = new HashSet<>();
    private ElapsedTime clock;
    private double time = 0;
    private double lastTime = 0;
    public Scheduler() {
        this(new ElapsedTime());
    }
    public Scheduler(ElapsedTime clock) {
        this.clock = clock;
    }
    public double run(boolean active) {
        time = clock.seconds();
        for (Subsystem subsystem : subsystems.keySet()) {
            subsystem.update(time, active);
        }
        if (active) {
            for (Listener listener : listeners) {
                if (listener.ready()) {
                    schedule(listener.getCommand());
                }
            }
            for (Command command : commands) {
                if (command.done(time)) {
                    for (Subsystem subsystem : command.getSubsystems()) {
                        subsystems.put(subsystem, null);
                    }
                    command.end(time, false);
                    finished.add(command);
                } else {
                    command.run(time);
                }
            }
            for (Command command : added) {
                if (!finished.contains(command)) {
                    commands.add(command);
                }
            }
            commands.removeAll(finished);
            added.clear();
            finished.clear();
        }
        return -lastTime + (lastTime = time);
    }
    public boolean schedule(Command command) {
        HashSet<Command> toCancel = new HashSet<>();
        if (command == null || commands.contains(command)) {
            return false;
        }
        for (Subsystem subsystem : command.getSubsystems()) {
            if (!subsystems.containsKey(subsystem)) {
                throw new IllegalArgumentException("Unregistered subsystem");
            } else if (using(subsystem)) {
                if (subsystems.get(subsystem).cancelable) {
                    toCancel.add(subsystems.get(subsystem));
                } else {
                    return false;
                }
            }
        }
        cancel(toCancel.toArray(new Command[0]));
        for (Subsystem subsystem : command.getSubsystems()) {
            subsystems.put(subsystem, command);
        }
        command.init(time);
        added.add(command);
        return true;
    }
    public void cancel(Command... toCancel) {
        for (Command command : toCancel) {
            if (commands.contains(command)) {
                for (Subsystem subsystem : command.getSubsystems()) {
                    subsystems.put(subsystem, null);
                }
                command.end(time, true);
                finished.add(command);
            }
        }
    }
    public void register(Subsystem... toAdd) {
        for (Subsystem subsystem : toAdd) {
            subsystems.put(subsystem, null);
        }
    }
    public void unregister(Subsystem... toRemove) {
        for (Subsystem subsystem : toRemove) {
            if (using(subsystem)) {
                throw new IllegalArgumentException("Subsystem in use by a command");
            }
            subsystems.remove(subsystem);
        }
    }
    public boolean using(Subsystem system) {
        return subsystems.get(system) != null;
    }
    public void addListener(Listener... toAdd) {
        listeners.addAll(Arrays.asList(toAdd));
    }
    public void removeListener(Listener... toRemove) {
        listeners.removeAll(Arrays.asList(toRemove));
    }
    public Set<Command> getCommands() {
        return commands;
    }
    public Set<Subsystem> getSubsystems() {
        return subsystems.keySet();
    }
    public Set<Listener> getListeners() {
        return listeners;
    }
}
