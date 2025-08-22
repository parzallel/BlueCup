import threading
import asyncio
import log


with open("api/gear.txt" , "w") as file:
    file.write("3")


logger = log.getLogger(__name__)


async def main():
    # These files are imported here to set the logging format
    from robot_core import robot
    from api import AsyncMessageThread

    message_thread = AsyncMessageThread()
    robot_thread = threading.Thread(target=robot.start, daemon=True)

    try:
        # Start the background tasks. This method returns immediately.
        await message_thread.start()
        robot_thread.start()

        # This is the crucial part. We wait on a future that never
        # completes, keeping the program alive until we interrupt it.
        print("--- Program running. Press Ctrl+C to stop. ---")
        await asyncio.Future()

    except asyncio.CancelledError:
        logger.info("\nAsyncio task was cancelled.")
    except KeyboardInterrupt:
        logger.info("\nKeyboard interrupt received.")
    finally:
        print("Shutting down...")
        # Ensure a graceful shutdown by calling your stop method
        message_thread.stop()
        # Give a moment for cancellation messages to be processed
        await asyncio.sleep(0.1)
        print("Shutdown complete.")

if __name__ == "__main__":
    asyncio.run(main())
