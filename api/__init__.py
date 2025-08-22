import asyncio
import log

from .mavlink import mavlink, client
from . import message_handler, message_builder


logger = log.getLogger('api')


class AsyncMessageThread:
    def __init__(self):
        super().__init__()
        self.running = False
        self.connected = False
        self.tasks = []

    async def periodic_task(self, task_id: str, interval: float, action):
        """
        A self-correcting asynchronous task that runs periodically.
        It catches asyncio.CancelledError to handle graceful shutdown.
        """
        import asyncio
        import time
        logger.debug(
            f"STARTING -> Task '{task_id}' with {interval:.3f}s interval.")
        while True:
            start_time = time.perf_counter()

            await action()

            # Self-correct the sleep time to prevent drift
            elapsed = time.perf_counter() - start_time
            sleep_for = max(0, interval - elapsed)

            try:
                # The main wait. This is where other tasks get to run.
                await asyncio.sleep(sleep_for)
            except asyncio.CancelledError:
                # This block executes when task.cancel() is called on this task.
                logger.debug(f"CANCELLED -> Task '{task_id}'")
                break  # Exit the loop to end the task.

    async def recv_msg_task(self):
        while self.running:
            try:
                msg = await client.recv_msg()

                if not msg:
                    logger.warning("receiver returned no message.")
                    continue

                if msg.id in message_handler.handlers:
                    asyncio.create_task(message_handler.handlers[msg.id](msg))
                else:
                    logger.warning(
                        f"Unhandled message: {mavlink.mavlink_map[msg.id].msgname if msg.id in mavlink.mavlink_map else 'Unknown'} (ID: {msg.id})")
            except Exception as e:
                logger.exception(f"Error in recv_msg_task: {e}")

    async def start(self):
        """Start the message thread"""
        logger.info("Starting message thread")
        if self.running:
            logger.warning("Message thread is already running")
            return

        # Clear existing tasks to avoid duplicates
        for task in self.tasks:
            task.cancel()
        self.tasks = []

        self.running = True

        # Start the periodic tasks
        for event_id, (interval, action) in message_builder.events.items():
            task = asyncio.create_task(self.periodic_task(
                str(event_id), interval, action))
            self.tasks.append(task)

        # Start the message receiving loop
        receiver_task = asyncio.create_task(self.recv_msg_task())
        self.tasks.append(receiver_task)

    def stop(self):
        """Stop the message thread by cancelling only its own tasks."""
        if not self.tasks:
            logger.warning("No message thread tasks to stop.")
            return

        logger.info(f"Stopping {len(self.tasks)} message thread tasks...")
        self.running = False

        # Cancel only the tasks this class created
        for task in self.tasks:
            task.cancel()

        # Clear the list
        self.tasks = []
        logger.info("Message thread tasks cancelled.")
