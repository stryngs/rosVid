from typing import Optional

class ChainEncoder:
    """Work in progress"""
    def __init__(self, format = 'BGR', height = 480, width = 640):
        self.format = format
        self.height = height
        self.width = width


    def build(self):
        """Build the string"""
        return (f'video/x-raw,format={self.format},width={self.width},height={self.height}')


    def update(self, format = None, height = None, width = None):
        """Update the build"""
        return ChainEncoder(format = self.format if format is None else format,
                            height = self.height if height is None else height,
                            width = self.width if width is None else width)



class ChainDecoder:
    """Build and update for decoding chains"""
    def __init__(self, decoder = 'avdec_h264', format = 'BGR', height = 480, width = 640):
        self.decoder = decoder
        self.format = format
        self.height = height
        self.width = width
        if self.decoder.endswith('h265'):
            self.depay = 'rtph265depay'
            self.parse = 'h265parse'
        else:
            self.depay = 'rtph264depay'
            self.parse = 'h264parse'


    def build(self):
        """Build the string"""
        return (f'latency=0 drop-on-latency=true ! '
                f'{self.depay} ! {self.parse} ! {self.decoder} ! videoconvert ! videoscale ! '
                f'video/x-raw,format={self.format},width={self.width},height={self.height} ! '
                f'appsink name=appsink emit-signals=true sync=false drop=true max-buffers=1')


    def update(self, **kwargs):
        """Update the build"""
        return ChainDecoder(decoder = kwargs.get('decoder', self.decoder),
                            format = kwargs.get('format', self.format),
                            height = kwargs.get('height', self.height),
                            width = kwargs.get('width', self.width))



