/**
 * @author programming@first857.org | first@jacobdixon.us (Jacob Dixon)
 * @version 1.0a
 * @since 2020-03-03
 */

package org.first857.utils;

public class Maths {

    public static double limit(double d, double max) {
        if (d > max) {
            d = max;
        } else if (d < -max) {
            d = -max;
        }
        return d;
    }

    public static double limit(double d, double limit, double multiplier) {
        d = d * multiplier;
        
        if (d > limit) {
            d = limit;
        } else if (d < -limit) {
            d = -limit;
        }
        return d;
    }

    public static double deadband(double d, double deadband){
        if(d < deadband && d > -deadband){
            d = 0.0;
        } else if (d > -deadband && d < deadband){
            d = 0.0;
        }

        return d;
    }

}