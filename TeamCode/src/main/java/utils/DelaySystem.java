package utils;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.BooleanSupplier;

public class DelaySystem {
    List<DelayData> delays = new ArrayList<>();
    List<ConditionalDelayData> conditionalDelays = new ArrayList<>();

    class DelayData {
        public long startTime;
        public long delay;
        public DelayCallback callback;

        public DelayData(long delay, DelayCallback callback) {
            startTime = System.currentTimeMillis();
            this.delay = delay;
            this.callback = callback;
        }
    }

    class ConditionalDelayData {
        public BooleanSupplier equality;
        public DelayCallback callback;

        public ConditionalDelayData(BooleanSupplier equality, DelayCallback callback) {
            this.equality = equality;
            this.callback = callback;
        }
    }

    public void Update() {
        long time = System.currentTimeMillis();
        Iterator<DelayData> delayIterator = delays.iterator();
        while (delayIterator.hasNext()) {
            DelayData delay = delayIterator.next();
            if (time - delay.startTime > delay.delay) {
                delayIterator.remove();
                delay.callback.fire();
            }
        }
        Iterator<ConditionalDelayData> conditionalIterator = conditionalDelays.iterator();
        while (conditionalIterator.hasNext()) {
            ConditionalDelayData conditionalDelay = conditionalIterator.next();
            if (conditionalDelay.equality.getAsBoolean()) {
                conditionalIterator.remove();
                conditionalDelay.callback.fire();
            }
        }
    }

    public interface DelayCallback {
        void fire();
    }

    public void CreateDelay(long delay, DelayCallback callback) {
        delays.add(new DelayData(delay, callback));
    }

    public void CreateConditionalDelay(BooleanSupplier equality, DelayCallback callback) {
        conditionalDelays.add(new ConditionalDelayData(equality, callback));
    }
}
