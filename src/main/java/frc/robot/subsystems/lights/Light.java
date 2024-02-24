package frc.robot.subsystems.lights;

import java.util.ArrayList;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Light extends SubsystemBase {
    private CANdle candle;

    private static Light light;    
    public static Light getInstance() {
        if (light == null) {
            light = new Light();
        }
        return light;
    }

    public boolean command = false;
    private ArrayList<Animations> animationsList = new ArrayList<Animations>();
    //Honestly some of the best logic code I've ever written. TODO:Document this later
    public Light() {
        this.candle = new CANdle(99);
    }
    
    public void setAnimation(Animations animations) {
        if (animations.time == 0) {
            configAnimation(animations.animation);
            return;
        }
        animationsList.add(animations);
    }

    public void setAnimation(Animations[] animations) {
        for (Animations animations2 : animations) {
            setAnimation(animations2);
        }
    }

    private Animation lastAnimation;
    private void configAnimation(Animation animation) {
        if (animation == lastAnimation) return;
        candle.animate(animation);
        lastAnimation = animation;
    }

    int flag = 0;
    double start = 0;
    @Override
    public void periodic() {
        if (animationsList.isEmpty()) {
            return;
        }
        switch (flag) {
            case 0:
                start = Timer.getFPGATimestamp();
                configAnimation(animationsList.get(0).animation);
                flag = 1;
                break;
            case 1:
                if (animationsList.get(0).time <= Timer.getFPGATimestamp() - start) {
                    animationsList.remove(0);
                    flag = 0;
                    configAnimation(Animations.DEFAULT.animation);
                    break;
                }
        }
    }
}