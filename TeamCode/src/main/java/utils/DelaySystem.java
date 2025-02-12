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
        public Runnable callback;

        public DelayData(long delay, Runnable callback) {
            startTime = System.currentTimeMillis();
            this.delay = delay;
            this.callback = callback;
        }
    }

    class ConditionalDelayData {
        public BooleanSupplier equality;
        public Runnable callback;
        public long startTime;
        public long timeout = -1;
        public Runnable timeoutCallback;

        public ConditionalDelayData(BooleanSupplier equality, Runnable callback) {
            this.equality = equality;
            this.callback = callback;
        }

        public ConditionalDelayData(BooleanSupplier equality, Runnable callback, long timeout, Runnable timeoutCallback) {
            startTime = System.currentTimeMillis();
            this.equality = equality;
            this.callback = callback;
            this.timeout = timeout;
            this.timeoutCallback = timeoutCallback;
        }
    }

    public void update() {
        long time = System.currentTimeMillis();
        Iterator<DelayData> delayIterator = delays.iterator();
        while (delayIterator.hasNext()) {
            DelayData delay = delayIterator.next();
            if (time - delay.startTime >= delay.delay) {
                delayIterator.remove();
                delay.callback.run();
            }
        }
        Iterator<ConditionalDelayData> conditionalIterator = conditionalDelays.iterator();
        while (conditionalIterator.hasNext()) {
            ConditionalDelayData delay = conditionalIterator.next();
            if (delay.equality.getAsBoolean()) {
                conditionalIterator.remove();
                delay.callback.run();
            }
            else if (delay.timeout > -1 && time - delay.startTime >= delay.timeout) {
                conditionalIterator.remove();
                delay.timeoutCallback.run();
            }
        }
    }

    public void createDelay(long delay, Runnable callback) {
        delays.add(new DelayData(delay, callback));
    }

    public void createConditionalDelay(BooleanSupplier equality, Runnable callback) {
        conditionalDelays.add(new ConditionalDelayData(equality, callback));
    }

    public void createConditionalDelay(BooleanSupplier equality, Runnable callback, long timeout, Runnable timeoutCallback) {
        conditionalDelays.add(new ConditionalDelayData(equality, callback, timeout, timeoutCallback));
    }
}
