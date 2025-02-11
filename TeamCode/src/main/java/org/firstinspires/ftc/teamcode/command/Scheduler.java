package org.firstinspires.ftc.teamcode.command;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
public class Scheduler {
    private Set<Command> commands = new HashSet<>();
    private Map<Subsystem, Command> subsystems = new HashMap<>();
    private Set<Listener> listeners = new HashSet<>();
    private Queue<Command> added = new ArrayDeque<>();
    private LinkedHashMap<Command, Boolean> finished = new LinkedHashMap<>();
    public double lastTime = System.nanoTime() * 1e-9;
    public double run(boolean active) {
        double time = System.nanoTime() * 1e-9;
        for (Subsystem subsystem : subsystems.keySet()) {
            subsystem.update(time, active);
        }
        if (active) {
            for (Listener listener : listeners) {
                if (listener.ready()) {
                    schedule(listener.getCommand());
                }
            }
            for (int i = added.size(); i > 0; i--) {
                Command command = added.poll();
                if (!finished.containsKey(command)) {
                    commands.add(command);
                    command.init(time);
                }
            }
            for (Command command : commands) {
                if (!finished.containsKey(command)) {
                    if (command.done(time)) {
                        for (Subsystem subsystem : command.getSubsystems()) {
                            subsystems.put(subsystem, null);
                        }
                        finished.put(command, false);
                    } else {
                        command.run(time);
                    }
                }
            }
            for (int i = finished.size(); i > 0; i--) {
                Command command = finished.keySet().iterator().next();
                commands.remove(command);
                command.end(time, finished.get(command));
                finished.remove(command);
            }
        }
        return -lastTime + (lastTime = time);
    }
    public boolean schedule(Command command) {
        if (command == null) {
            return false;
        }
        List<Command> toCancel = new ArrayList<>();
        for (Subsystem subsystem : command.getSubsystems()) {
            if (!subsystems.containsKey(subsystem)) {
                throw new IllegalArgumentException("Unregistered subsystem");
            } else if (using(subsystem)) {
                Command cmd2 = subsystems.get(subsystem);
                if (cmd2.cancelable) {
                    toCancel.add(cmd2);
                } else {
                    return false;
                }
            }
        }
        cancel(toCancel.toArray(new Command[0]));
        for (Subsystem subsystem : command.getSubsystems()) {
            subsystems.put(subsystem, command);
        }
        added.add(command);
        return true;
    }
    public void cancel(Command... toCancel) {
        for (Command command : toCancel) {
            if (!finished.containsKey(command)) {
                for (Subsystem subsystem : command.getSubsystems()) {
                    subsystems.put(subsystem, null);
                }
                finished.put(command, true);
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
