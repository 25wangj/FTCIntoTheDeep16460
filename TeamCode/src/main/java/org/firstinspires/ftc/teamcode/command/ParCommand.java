package org.firstinspires.ftc.teamcode.command;

import java.util.ArrayList;
import java.util.List;

public class ParCommand extends Command {
    private Command[] commands;
    private boolean[] dones;
    private boolean[] justDones;
    private int numDone;
    public ParCommand(Command... commands) {
        List<Command> list = new ArrayList<>();
        for (Command command : commands) {
            if (command != null) {
                for (Subsystem subsystem : command.getSubsystems()) {
                    if (subsystems.contains(subsystem)) {
                        throw new IllegalArgumentException("Subsystem used twice");
                    }
                    subsystems.add(subsystem);
                }
                cancelable = cancelable && command.cancelable;
                list.add(command);
            }
        }
        this.commands = list.toArray(new Command[0]);
    }
    @Override
    public void init(double time) {
        dones = new boolean[commands.length];
        justDones = new boolean[commands.length];
        numDone = 0;
        for (Command command : commands) {
            command.init(time);
        }
    }
    @Override
    public void run(double time) {
        for (int i = 0; i < commands.length; i++) {
            if (justDones[i]) {
                commands[i].end(time, false);
                justDones[i] = false;
            } else if (!dones[i]) {
                commands[i].run(time);
            }
        }
    }
    @Override
    public void end(double time, boolean canceled) {
        if (canceled) {
            for (int i = 0; i < commands.length; i++) {
                if (!dones[i]) {
                    commands[i].end(time, true);
                }
            }
        }
        for (int i = 0; i < commands.length; i++) {
            if (justDones[i]) {
                commands[i].end(time, false);
            }
        }
    }
    @Override
    public boolean done(double time) {
        for (int i = 0; i < commands.length; i++) {
            if (!dones[i] && commands[i].done(time)) {
                justDones[i] = true;
                dones[i] = true;
                numDone++;
            }
        }
        return numDone == commands.length;
    }
}
