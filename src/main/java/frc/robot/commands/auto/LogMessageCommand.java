package frc.robot.commands.auto;

import frc.robot.commands.LoggingCommand;

public class LogMessageCommand extends LoggingCommand {

    private final String msg;

    public LogMessageCommand(String msg) {
        this.msg = msg;
    }

    @Override
    public boolean isFinished() {
        log(msg);
        return true;
    }
}
