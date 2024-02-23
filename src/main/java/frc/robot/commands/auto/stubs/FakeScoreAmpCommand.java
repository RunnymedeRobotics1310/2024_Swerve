package frc.robot.commands.auto.stubs;

import frc.robot.commands.LoggingCommand;

public class FakeScoreAmpCommand extends LoggingCommand {
    public FakeScoreAmpCommand() {
        super();
    }

    @Override
    public void execute() {
        log("HE SHOOTS, HE SCORES!");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}