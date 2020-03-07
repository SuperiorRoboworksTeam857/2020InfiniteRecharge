package org.first857.utils;

import edu.wpi.first.wpilibj.Joystick;

public class Safety {

    public static boolean checkJoystickValidity(int port, String nameSnippet){
        Joystick s = new Joystick(port);
        boolean check = s.getName().toLowerCase().contains(nameSnippet.toLowerCase());
        if(!check) System.err.println("WARNING: Joystick " + port + " was detected as a different type than expected, check ports. Expected: " + nameSnippet + " Got: " + s.getName());
        s = null;
        return check;
    }

    public static boolean checkJoystickValidity(Joystick joystick, String nameSnippet){
        boolean check = joystick.getName().toLowerCase().contains(nameSnippet.toLowerCase());
        if(!check) System.err.println("WARNING: Joystick " + joystick.getPort() + " was detected as a different type than expected, check ports. Expected: " + nameSnippet + " Got: " + joystick.getName());
        return check;
    }
}