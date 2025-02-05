package org.team1540.robot2025.util.math;

import java.util.LinkedList;

// hopefully we won't have skill issues with this class anymore (2023orwil moment)
public class AverageFilter {
    private final LinkedList<Double> items = new LinkedList<>();
    private double sum = 0;
    private final int size;

    public AverageFilter(int size) {
        this.size = size;
    }

    public void clear() {
        items.clear();
        sum = 0;
    }

    public void add(double value) {
        items.addLast(value);
        sum += value;
        if (items.size() > this.size) {
            sum -= items.removeFirst();
        }
    }

    public double getAverage() {
        return sum / (double) items.size();
    }
}
