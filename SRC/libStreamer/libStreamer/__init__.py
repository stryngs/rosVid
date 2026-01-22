#!/usr/bin/env python3

import cv2
from typing import Dict, Optional

class Config:
    """Create a configuration for a given input source"""
    def __init__(self,
                 controls = None,
                 device = '/dev/video0',
                 fourcc = None,
                 fps = 30,
                 height = 480,
                 width = 640):
        self.controls = controls
        self.device = device
        self.fps = int(fps)
        self.fourcc = fourcc
        self.height = int(height)
        self.width = int(width)
        

    def update(self, updates):
        """Return a copy with provided keys applied if present."""
        return Config(controls = updates.get('controls', self.controls),
                      device = updates.get('device', self.device),
                      fourcc = updates.get('fourcc', self.fourcc),
                      fps = int(updates.get('fps', self.fps)),
                      height = int(updates.get('height', self.height)),
                      width = int(updates.get('width', self.width)))
