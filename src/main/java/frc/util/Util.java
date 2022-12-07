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

}
