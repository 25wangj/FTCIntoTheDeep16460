package org.firstinspires.ftc.teamcode.command;

import java.util.ArrayList;

public class SeqCommand extends Command {
    private Command[] commands;
    private int index;
    public SeqCommand(Command... commands) {
        ArrayList<Command> list = new ArrayList<>();
        for (Command command : commands) {
            if (command != null) {
                subsystems.addAll(command.getSubsystems());
                cancelable = cancelable && command.cancelable;
                list.add(command);
            }
        }
        this.commands = list.toArray(new Command[0]);
    }
    @Override
    public void init(double time) {
        index = 0;
        commands[0].init(time);
    }
    @Override
    public void run(double time) {
        if (index != commands.length - 1 && commands[index].done(time)) {
            commands[index].end(time, false);
            index++;
            commands[index].init(time);
        } else {
            commands[index].run(time);
        }
    }
    @Override
    public void end(double time, boolean canceled) {
        commands[index].end(time, canceled);
    }
    @Override
    public boolean done(double time) {
        return index == commands.length - 1 && commands[index].done(time);
    }
}
