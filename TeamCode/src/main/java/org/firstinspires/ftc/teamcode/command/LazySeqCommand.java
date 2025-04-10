package org.firstinspires.ftc.teamcode.command;
import java.util.ArrayList;
import java.util.function.Supplier;
public class LazySeqCommand extends Command {
    private Command curr;
    private Supplier<Command>[] commands;
    private int index;
    public LazySeqCommand(Supplier<Command>... commands) {
        ArrayList<Supplier<Command>> list = new ArrayList<>();
        for (Supplier<Command> supplier : commands) {
            Command test = supplier.get();
            if (test != null) {
                list.add(supplier);
                subsystems.addAll(test.getSubsystems());
                cancelable = cancelable && test.cancelable;
            }
        }
        this.commands = list.toArray(new Supplier[0]);
    }
    @Override
    public void init(double time) {
        index = 0;
        curr = commands[0].get();
        curr.init(time);
    }
    @Override
    public void run(double time) {
        if (index != commands.length - 1 && curr.done(time)) {
            curr.end(time, false);
            index++;
            curr = commands[index].get();
            curr.init(time);
        } else {
            curr.run(time);
        }
    }
    @Override
    public void end(double time, boolean canceled) {
        curr.end(time, canceled);
    }
    @Override
    public boolean done(double time) {
        return index == commands.length - 1 && curr.done(time);
    }
}
