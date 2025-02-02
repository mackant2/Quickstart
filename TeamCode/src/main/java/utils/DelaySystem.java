package utils;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class DelaySystem {
    List<DelayData> delays = new ArrayList<>();

    static class DelayData {
        public long startTime;
        public long delay;
        public DelayCallback callback;

        public DelayData(long delay, DelayCallback callback) {
            startTime = System.currentTimeMillis();
            this.delay = delay;
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
    }

    public interface DelayCallback {
        void fire();
    }

    public void CreateDelay(long delay, DelayCallback callback) {
        delays.add(new DelayData(delay, callback));
    }
}
