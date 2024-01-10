package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;

/**
 * The Base Runnymede Command implements command helpers to aid with logging
 */
public class RunnymedeCommand extends Command {

    SimpleDateFormat START_TIMESTAMP_FMT = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");

    protected long  initializeTime = 0;
    private String  finishReason   = null;

    List<Subsystem> subsystemList  = new ArrayList<>();

    /**
     * Default implementation automatically logs start of command with required subsystems
     */
    public void initialize() {
        logCommandStart();
    }

    public void logCommandStart() {

        logCommandStart(null, new Subsystem[0]);
    }

    public void logCommandStart(String msg) {

        logCommandStart(msg, new Subsystem[0]);
    }

    public void logCommandStart(String commandParms, Subsystem... subsystemList) {

        this.subsystemList.clear();
        finishReason   = null;
        initializeTime = 0;

        // Capture the subsystem list associated with this command
        if (subsystemList != null && subsystemList.length > 0) {
            this.subsystemList.addAll(Arrays.asList(subsystemList));
        }
        else {
            this.subsystemList.addAll(getRequirements());
        }

        logCommandState("STARTING", commandParms, true);

        // Set the initialize time after logging of the start message.
        initializeTime = System.currentTimeMillis();
    }

    /**
     * Default implementation logs command end
     * @param interrupted whether the command was interrupted/canceled
     */
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }

    public void logCommandEnd(boolean interrupted) {

        logCommandEnd(interrupted, null);
    }

    public void logCommandEnd(boolean interrupted, String endMsg) {

        String state = "ENDED";

        if (interrupted) {
            state = "INTERRUPTED";
        }

        logCommandState(state, endMsg, true);
    }

    public void logStateTransition(String transition) {
        logStateTransition(transition, null);
    }

    public void logStateTransition(String transition, boolean logSubsystems) {
        logStateTransition(transition, null, logSubsystems);
    }

    public void logStateTransition(String transition, String msg) {
        logStateTransition(transition, msg, false);
    }

    public void logStateTransition(String transition, String msg, boolean logSubsystems) {
        logCommandState(transition, msg, logSubsystems);
    }

    public void log(String msg) {
        logCommandState(null, msg, false);
    }

    public void log(String msg, boolean logSubsystems) {
        logCommandState(null, msg, logSubsystems);
    }

    private void logCommandState(String state, String msg, boolean logSubsystems) {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName());

        if (state != null) {
            sb.append(" : ").append(state);
        }

        if (initializeTime == 0) {
            sb.append(" at ").append(START_TIMESTAMP_FMT.format(new Date()));
        } else {
            sb.append(" at ").append(System.currentTimeMillis() - initializeTime).append("ms");
        }

        if (finishReason != null) {
            sb.append(" : ").append(finishReason);
        }

        if (msg != null) {
            sb.append(" : ").append(msg);
        }

        if (logSubsystems) {
            // Print the subsystems as passed in on the command start
            for (Subsystem subsystem : subsystemList) {
                if (subsystem != null) {
                    sb.append("\n   ").append(subsystem.toString());
                }
            }
        }

        System.out.println(sb.toString());
    }

    public void setFinishReason(String finishReason) {
        this.finishReason = finishReason;
    }


}
