package utils;

import java.util.ArrayList;
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
        List<DelayData> delaysToRemove = new ArrayList<>();
        for (DelayData delayData : delays) {
            if (time - delayData.startTime > delayData.delay) {
                delayData.callback.fire();
                delaysToRemove.add(delayData);
            }
        }
        for (DelayData delayData : delaysToRemove) {
            delays.remove(delayData);
        }
    }

    public interface DelayCallback {
        void fire();
    }

    public void CreateDelay(long delay, DelayCallback callback) {
        delays.add(new DelayData(delay, callback));
    }
}
