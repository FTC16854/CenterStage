package org.firstinspires.ftc.teamcode;

public class Toggle {
    public boolean output;
    public boolean button_press;

    Toggle(boolean button_of_toggle) {
        button_press = button_of_toggle;
    }

    public boolean Toggle_Button(){
    if (button_press){
        output = output;
    }
    return output;
    }

}
