"""Splits an MJPEG-over-HTTP stream (multipart/x-mixed-replace) into JPEG frames."""

import re

SOI = b'\xff\xd8'
EOI = b'\xff\xd9'
CONTENT_LENGTH = re.compile(rb'content-length:\s*(\d+)', re.IGNORECASE)
MAX_BUFFER = 8 * 1024 * 1024  # drop garbage rather than grow without bound


class MjpegParser:
    """Feed it bytes as they arrive; it returns each complete JPEG.

    Uses the part's Content-Length when the server sends one (uStreamer does), so a JPEG that
    happens to contain an end-of-image marker inside (e.g. an EXIF thumbnail) is not cut short.
    Otherwise it falls back to the start/end-of-image markers.
    """

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer += data
        frames = []
        while True:
            start = self.buffer.find(SOI)
            if start < 0:
                if len(self.buffer) > MAX_BUFFER:
                    del self.buffer[:-1]
                break
            match = None
            for match in CONTENT_LENGTH.finditer(self.buffer, 0, start):
                pass  # keep the last header before this image
            if match is not None:
                end = start + int(match.group(1))
            else:
                marker = self.buffer.find(EOI, start + 2)
                end = marker + 2 if marker >= 0 else -1
            if end < 0 or end > len(self.buffer):
                if len(self.buffer) > MAX_BUFFER:
                    del self.buffer[:]
                break
            frames.append(bytes(self.buffer[start:end]))
            del self.buffer[:end]
        return frames
