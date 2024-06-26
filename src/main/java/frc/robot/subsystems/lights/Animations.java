package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import frc.robot.Constants;

public enum Animations {
    DEFAULT(new StrobeAnimation(255, 255, 255, 0, 1, Constants.NUM_LEDS), 0),
    COLLECTING(new StrobeAnimation(255, 255, 0, 0, 0.5, Constants.NUM_LEDS), 0),
    INTAKE_STAGE_1(new StrobeAnimation(255, 0, 255, 0, 1, Constants.NUM_LEDS), 0),
    INTAKE_STAGE_2(new StrobeAnimation(0, 0, 255, 0, 1, Constants.NUM_LEDS), 0),
    AIMING(new StrobeAnimation(255, 0, 0, 0, 0.5, Constants.NUM_LEDS), 0),
    SHOT_READY(new StrobeAnimation(0, 255, 0, 0, 1, Constants.NUM_LEDS), 0),
    JADEN_U_HAVE_A_NOTE(new StrobeAnimation(255, 0, 255, 0, 0.8, Constants.NUM_LEDS), 0),
    CLIMB(new RainbowAnimation(), 0),
    SHOT_SPECIAL(new StrobeAnimation(0, 255, 0, 0, 0.5, Constants.NUM_LEDS), 0),
    PRE_INTAKE(new StrobeAnimation(0, 255, 0, 0, 1, Constants.NUM_LEDS), 0), 
    TEAL(new StrobeAnimation(0,255,200,0,1,Constants.NUM_LEDS),0);

    
    public Animation animation;
    public double time;
    private Animations(Animation animation, double time) {
        this.animation = animation;
        this.time = time;
    }

    
}
