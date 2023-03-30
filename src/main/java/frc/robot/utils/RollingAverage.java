package frc.robot.utils;

public class RollingAverage {
    private int size;
    private double total = 0;
    private int index = 0;
    private double samples[];
    public RollingAverage(int size) {
        this.size = size;
        samples = new double[size];
        for (int i = 0; i < size; i++) samples[i] = 0;
    }

    public void add(double x) {
        total -= samples[index];
        samples[index] = x;
        total += x;
        if(++index == size) index = 0;
    }

    public double getAverage() {
        return total / size;
    }

}