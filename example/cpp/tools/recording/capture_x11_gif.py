#!/usr/bin/env python3

import argparse
import ctypes
import time
from pathlib import Path

from PIL import Image


class XImage(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("xoffset", ctypes.c_int),
        ("format", ctypes.c_int),
        ("data", ctypes.c_void_p),
        ("byte_order", ctypes.c_int),
        ("bitmap_unit", ctypes.c_int),
        ("bitmap_bit_order", ctypes.c_int),
        ("bitmap_pad", ctypes.c_int),
        ("depth", ctypes.c_int),
        ("bytes_per_line", ctypes.c_int),
        ("bits_per_pixel", ctypes.c_int),
        ("red_mask", ctypes.c_ulong),
        ("green_mask", ctypes.c_ulong),
        ("blue_mask", ctypes.c_ulong),
    ]


class X11Capture:
    def __init__(self):
        self.x11 = ctypes.CDLL("libX11.so.6")
        self.x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.display = self.x11.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError("Cannot open X11 display")
        self.x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.x11.XCloseDisplay.restype = ctypes.c_int

        self.x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        self.x11.XDefaultRootWindow.restype = ctypes.c_ulong
        self.x11.XQueryTree.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),
            ctypes.POINTER(ctypes.c_uint),
        ]
        self.x11.XQueryTree.restype = ctypes.c_int
        self.x11.XFetchName.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_char_p),
        ]
        self.x11.XFetchName.restype = ctypes.c_int
        self.x11.XFree.argtypes = [ctypes.c_void_p]
        self.x11.XFree.restype = ctypes.c_int
        self.x11.XGetGeometry.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_uint),
            ctypes.POINTER(ctypes.c_uint),
            ctypes.POINTER(ctypes.c_uint),
            ctypes.POINTER(ctypes.c_uint),
        ]
        self.x11.XGetGeometry.restype = ctypes.c_int
        self.x11.XGetImage.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_uint,
            ctypes.c_uint,
            ctypes.c_ulong,
            ctypes.c_int,
        ]
        self.x11.XGetImage.restype = ctypes.POINTER(XImage)
        self.x11.XDestroyImage.argtypes = [ctypes.POINTER(XImage)]
        self.x11.XDestroyImage.restype = ctypes.c_int

    def close(self):
        if self.display:
            self.x11.XCloseDisplay(self.display)
            self.display = None

    def _children(self, window):
        root = ctypes.c_ulong()
        parent = ctypes.c_ulong()
        children = ctypes.POINTER(ctypes.c_ulong)()
        count = ctypes.c_uint()
        if not self.x11.XQueryTree(
            self.display,
            window,
            ctypes.byref(root),
            ctypes.byref(parent),
            ctypes.byref(children),
            ctypes.byref(count),
        ):
            return []
        result = [children[index] for index in range(count.value)]
        if children:
            self.x11.XFree(children)
        return result

    def _name(self, window):
        name = ctypes.c_char_p()
        if self.x11.XFetchName(self.display, window, ctypes.byref(name)):
            value = name.value.decode(errors="replace") if name.value else ""
            if name:
                self.x11.XFree(name)
            return value
        return ""

    def find_window(self, title):
        root = self.x11.XDefaultRootWindow(self.display)
        pending = [root]
        while pending:
            window = pending.pop()
            if title.lower() in self._name(window).lower():
                return window
            pending.extend(self._children(window))
        raise RuntimeError(f"No X11 window title contains: {title}")

    def geometry(self, window):
        root = ctypes.c_ulong()
        x = ctypes.c_int()
        y = ctypes.c_int()
        width = ctypes.c_uint()
        height = ctypes.c_uint()
        border = ctypes.c_uint()
        depth = ctypes.c_uint()
        if not self.x11.XGetGeometry(
            self.display,
            window,
            ctypes.byref(root),
            ctypes.byref(x),
            ctypes.byref(y),
            ctypes.byref(width),
            ctypes.byref(height),
            ctypes.byref(border),
            ctypes.byref(depth),
        ):
            raise RuntimeError("XGetGeometry failed")
        return width.value, height.value

    def frame(self, window):
        width, height = self.geometry(window)
        image_ptr = self.x11.XGetImage(
            self.display,
            window,
            0,
            0,
            width,
            height,
            ctypes.c_ulong(-1).value,
            2,
        )
        if not image_ptr:
            raise RuntimeError("XGetImage failed")
        image = image_ptr.contents
        if image.bits_per_pixel != 32:
            self.x11.XDestroyImage(image_ptr)
            raise RuntimeError(
                f"Unsupported X11 pixel format: {image.bits_per_pixel} bpp"
            )
        size = image.bytes_per_line * image.height
        pixels = ctypes.string_at(image.data, size)
        frame = Image.frombuffer(
            "RGB",
            (image.width, image.height),
            pixels,
            "raw",
            "BGRX",
            image.bytes_per_line,
            1,
        ).copy()
        self.x11.XDestroyImage(image_ptr)
        return frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--title", default="MuJoCo")
    parser.add_argument("--duration", type=float, default=13.0)
    parser.add_argument("--fps", type=float, default=8.0)
    parser.add_argument("--width", type=int, default=800)
    args = parser.parse_args()

    capture = X11Capture()
    try:
        window = capture.find_window(args.title)
        frames = []
        frame_period = 1.0 / args.fps
        deadline = time.monotonic() + args.duration
        next_frame = time.monotonic()
        while time.monotonic() < deadline:
            frame = capture.frame(window)
            if frame.width > args.width:
                height = round(frame.height * args.width / frame.width)
                frame = frame.resize((args.width, height), Image.Resampling.LANCZOS)
            frames.append(frame)
            next_frame += frame_period
            time.sleep(max(0.0, next_frame - time.monotonic()))
    finally:
        capture.close()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    frame_duration_ms = round(1000.0 / args.fps)
    frames[0].save(
        args.output,
        save_all=True,
        append_images=frames[1:],
        duration=frame_duration_ms,
        loop=0,
        optimize=True,
    )
    print(f"Saved {len(frames)} frames to {args.output}")


if __name__ == "__main__":
    main()
