package frc.util;

public class Util {

    public static String classToJsonName(Class<?> clazz) {
        if (clazz.isAnonymousClass()) {
            return "anonymous" + clazz.getSuperclass().getSimpleName();
        }
        String className = clazz.getSimpleName();
        // Make first character lowercase to match JSON conventions
        return Character.toLowerCase(className.charAt(0)) + className.substring(1);
    }

    public static double handleDeadBand(double value, double deadBand) {
        return (Math.abs(value) > Math.abs(deadBand)) ? value : 0.0;
    }

    public static double clamp01(double value) {
        return clamp(value, -1.0, 1.0);
    }

    public static double clamp(double value, double minimum, double maximum) {
        return Math.min(maximum, Math.max(minimum, value));
    }
}
