package frc.robot.util;

import java.util.Objects;

/**
 * This is a logger that will remember the last thing it printed out, and will
 * suppress repeated statements to avoid flooding the log with repeat statements.
 */
public class Logger {

    public static final int SUPPRESS_REPEATS = 100;
    
    public static String makeMessage(Object... args) {
        StringBuilder builder = new StringBuilder();
        for (int i=0; i<args.length; i++) {
            builder.append(Objects.toString(args[i]));
        }
        return builder.toString();
    }

    private static String lastMessage = null;
    private static int lastRepeats = 0;

    public static void log(Object... args) {

        String message = makeMessage(args);

        if (lastMessage == null) {
            lastMessage = message;
            lastRepeats = 0;
            System.err.println(lastMessage);
        }
        else if (lastMessage.equals(message)) {
            lastRepeats += 1;
            if (lastRepeats % SUPPRESS_REPEATS == (SUPPRESS_REPEATS - 1)) {
                System.err.println(message+" ("+lastRepeats+" repeats)");
                lastRepeats = 0;
            }
        }
        else {
            if (lastRepeats > 0) {
                System.err.println(message+" ("+lastRepeats+" repeats)");
            }
            lastMessage = message;
            lastRepeats = 0;
            System.err.println(message);
       }
    }
}
