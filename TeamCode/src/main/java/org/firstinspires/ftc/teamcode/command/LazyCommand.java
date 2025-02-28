package org.firstinspires.ftc.teamcode.command;
import java.util.function.Supplier;
public class LazyCommand extends Command {
    private Supplier<Command> command;
    private Command instance = null;
    public LazyCommand(Supplier<Command> command) {
        this.command = command;
        subsystems = command.get().subsystems;
    }
    @Override
    public void init(double time) {
        instance = command.get();
        instance.init(time);
    }
    @Override
    public void run(double time) {
        instance.run(time);
    }
    @Override
    public void end(double time, boolean canceled) {
        instance.end(time, canceled);
    }
    @Override
    public boolean done(double time) {
        return instance.done(time);
    }
}
