package org.firstinspires.ftc.teamcode;

public class VirtualGamePad {
    protected double left_stick_x, left_stick_y, right_stick_x, right_stick_y, left_trigger, right_trigger;
    protected boolean x, y, a, b, left_bumper, right_bumper, left_stick_button, right_stick_button;
    protected boolean dpad_up, dpad_right, dpad_down, dpad_left;

    public void fromString(String s) {
        /** populate this from a string in format
         * "LX LY RX RY LT RT A B X Y LB RB LSB RSB DPU DPR DPD DPL"
         * Where LX, LY RX, RY, LT, and RT are 2-place doubles with values for left_stick_x,
         * left_stick_y, right_stick_x, right_stick_y, left_trigger, and right_trigger resp.
         *
         * A, B, X, Y, LB, RB, LSB, RSB are 1 or 0 for values of a, b, x, y, left_bumper,
         * right_bumper, left_stick_button, right_stick_button
         *
         * DPU, DPR, DPD, DPL are booleans for dpad_up, dpad_right, dpad_down, and dpad_left.
         */

        String[] values;

        try {
            values = s.split(" ");
            left_stick_x = Float.valueOf(values[0]);
            left_stick_y = Float.valueOf(values[1]);
            right_stick_x = Float.valueOf(values[2]);
            right_stick_y = Float.valueOf(values[3]);
            left_trigger = Float.valueOf(values[4]);
            right_trigger = Float.valueOf(values[5]);
            a = (Integer.valueOf(values[6])==1)? true:false;
            b = (Integer.valueOf(values[7])==1)? true:false;
            x = (Integer.valueOf(values[8])==1)? true:false;
            y = (Integer.valueOf(values[9])==1)? true:false;
            left_bumper = (Integer.valueOf(values[10])==1)? true:false;
            right_bumper = (Integer.valueOf(values[11])==1)? true:false;
            left_stick_button = (Integer.valueOf(values[12])==1)? true:false;
            right_stick_button = (Integer.valueOf(values[13])==1)? true:false;
            dpad_up = (Integer.valueOf(values[14])==1)? true:false;
            dpad_right = (Integer.valueOf(values[15])==1)? true:false;
            dpad_down = (Integer.valueOf(values[16])==1)? true:false;
            dpad_left = (Integer.valueOf(values[17])==1)? true:false;
        }
        finally {
            return;
        }

    }
}
