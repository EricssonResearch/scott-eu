package se.ericsson.cf.scott.sandbox.executor;

// TODO Andrew@2019-01-22: remove
/*
class PlanQueueRunnable implements Runnable {
    private final IQueue<Object> hcQueue;

    PlanQueueRunnable(final HazelcastInstance hc) {
        hcQueue = hc.getQueue("dummy");
    }

    @Override
    public void run() {
        PlanExecutorManager.log.debug("Starting a thread to process Hazelcast queue items");
        try {
            //noinspection InfiniteLoopStatement
            while (true) {
                final Object object = hcQueue.take();
                PlanExecutorManager.log.info("New object to process: {}", object);
            }
        } catch (InterruptedException e) {
            PlanExecutorManager.log.warn("Queue processing was interrupted, shutting down the bg thread");
        }
    }
}
*/
