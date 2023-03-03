package frc.robot.utils;

public class Toggle {
    private boolean state;
    private boolean buttonPressed = false;

    public Toggle(boolean defaultState){
        this.state = defaultState;
    }
    
    public boolean runToggle(boolean button){
        if (button && !this.buttonPressed){
            toggle();
            this.buttonPressed = true;
        } else if(!button && this.buttonPressed) {
            this.buttonPressed = false;
            return true;
        }
        return false;
    }

    public boolean getState(){
        return this.state;
    }

    public boolean setState(boolean state){
        this.state = state;
        return this.state;
    }
    
    private boolean toggle() {
        this.state = !this.state;
        return this.state;
    }
}