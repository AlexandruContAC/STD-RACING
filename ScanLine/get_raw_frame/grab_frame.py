import subprocess
import threading
import queue
import numpy as np
import cv2

# Pixy2's default raw frame resolution
WIDTH = 316
HEIGHT = 208

FRAME_BYTES = WIDTH * HEIGHT

# Queue with max size 1 — the reader always drops old frames, keeping only the latest
frame_queue = queue.Queue(maxsize=1)
stop_event = threading.Event()

def frame_reader(process):
    """Background thread: reads frames from the C++ process stdout as fast as possible."""
    while not stop_event.is_set():
        raw_bytes = process.stdout.read(FRAME_BYTES)
        if not raw_bytes or len(raw_bytes) != FRAME_BYTES:
            stop_event.set()
            break
        # Drop old frame if the display thread hasn't consumed it yet
        try:
            frame_queue.put_nowait(raw_bytes)
        except queue.Full:
            try:
                frame_queue.get_nowait()  # discard stale frame
            except queue.Empty:
                pass
            frame_queue.put_nowait(raw_bytes)

print("Calling C++ executable to grab frame...")

# 1. Run the compiled C++ executable silently
process = subprocess.Popen(
    ['./get_raw_frame'],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE
)

# 2. Start the background reader thread
reader_thread = threading.Thread(target=frame_reader, args=(process,), daemon=True)
reader_thread.start()

print("Press 'q' in the image window to close.")

try:
    while not stop_event.is_set():
        # 3. Get the latest frame (non-blocking; skip if none ready yet)
        try:
            raw_bytes = frame_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        # 4. Convert to NumPy array and reshape
        bayer_image = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((HEIGHT, WIDTH))

        # 5. Optionally demosaic (comment out for max speed / grayscale display)
        #rgb_image = cv2.cvtColor(bayer_image, cv2.COLOR_BayerBG2BGR)

        # 6. Display
        
        cv2.imshow("Pixy2 Raw Frame", bayer_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    stop_event.set()
    process.terminate()
    reader_thread.join(timeout=2)
    cv2.destroyAllWindows()