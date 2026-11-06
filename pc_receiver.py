"""
pc_receiver.py - receives the utterances sent by mic_capture_stream.ino and saves them as WAV files.

Run:        python pc_receiver.py
Needs:      Python 3 only (no extra packages).
Windows:    when the firewall asks, allow Python on PRIVATE networks.
PC address: run the command  ipconfig  and read "IPv4 Address". Write it in PC_IP in the sketch.
Protocol:   2 bytes length (little endian) + PCM audio (16 kHz, 16 bit, mono).
            A message with length 0 = end of the utterance.
Output:     recordings/utt_YYYYmmdd_HHMMSS.wav
"""

import array
import os
import socket
import struct
import threading
import time
import wave

HOST = "0.0.0.0"
PORT = 5005
SAMPLE_RATE = 16000
OUT_DIR = "recordings"
MIN_SECONDS = 0.3          # shorter recordings are ignored (clicks, bumps)


def recv_exact(conn, n):
    """Reads exactly n bytes, or returns None if the connection closed."""
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def save_utterance(chunks):
    pcm = b"".join(chunks)
    seconds = len(pcm) / 2 / SAMPLE_RATE
    if seconds < MIN_SECONDS:
        print(f"  ignored a very short recording ({seconds:.2f} s)")
        return
    samples = array.array("h")
    samples.frombytes(pcm[: len(pcm) // 2 * 2])
    peak = max((abs(s) for s in samples), default=0)

    os.makedirs(OUT_DIR, exist_ok=True)
    name = time.strftime("utt_%Y%m%d_%H%M%S") + ".wav"
    path = os.path.join(OUT_DIR, name)
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SAMPLE_RATE)
        w.writeframes(pcm)
    print(f"  saved {path}  ({seconds:.1f} s, peak {peak} of 32767)")
    if peak < 1500:
        print("  WARNING: very quiet recording. On the ESP32 lower MIC_SHIFT (13 or 12).")
    if peak >= 32000:
        print("  WARNING: the recording is clipping. On the ESP32 raise MIC_SHIFT (15).")


def handle(conn, addr):
    print(f"ESP32 connected from {addr[0]}")
    chunks = []
    try:
        while True:
            header = recv_exact(conn, 2)
            if header is None:
                break
            (length,) = struct.unpack("<H", header)
            if length == 0:
                print("end of utterance")
                save_utterance(chunks)
                chunks = []
                continue
            data = recv_exact(conn, length)
            if data is None:
                break
            chunks.append(data)
    except OSError as e:
        print(f"connection error: {e}")
    finally:
        conn.close()
        print("ESP32 disconnected")


def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    print(f"Listening on port {PORT}. Waiting for the ESP32... (Ctrl+C to stop)")
    try:
        while True:
            conn, addr = server.accept()
            threading.Thread(target=handle, args=(conn, addr), daemon=True).start()
    except KeyboardInterrupt:
        print("\nbye")
    finally:
        server.close()


if __name__ == "__main__":
    main()
