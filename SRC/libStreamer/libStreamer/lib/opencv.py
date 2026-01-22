import cv2

class CaptureHandler:
    """Handle capturing"""
    def __init__(self, config):
        self.config = config
        self.cv2 = cv2
        self.rList = ('device', 'fourcc', 'height', 'width')


    def applySettings(self, cap):
        """Apply the settings for a cap and return on the result"""
        if self.cv2 is None:
            raise RuntimeError('[!] No cv2')
        if self.config.fourcc:
            try:
                fourcc_val = self.cv2.VideoWriter_fourcc(*self.config.fourcc)
                cap.set(self.cv2.CAP_PROP_FOURCC, fourcc_val)
            except Exception as E:
                print(E)
        cap.set(self.cv2.CAP_PROP_FPS, self.config.fps)
        cap.set(self.cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        cap.set(self.cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        return cap.isOpened()


    def needsRestart(self, updates):
        """Checks for the restart using rList"""
        for key in self.rList:
            if key in updates and updates[key] is not None:
                if updates[key] != getattr(self.config, key):
                    return True
        return False


    def openCap(self):
        """Open the cap with the current config"""
        if self.cv2 is None:
            raise RuntimeError('[!] No cv2')
        cap = self.cv2.VideoCapture(self.config.device)
        ok = self.applySettings(cap)
        return cap if ok else None


    def updateChecks(self, updates, reCapture = None):
        """Handle updates and deal with restarts"""
        needsRestart = self.needsRestart(updates)
        updated = CaptureHandler(self.config.update(updates))
        reopened = None
        if needsRestart and reCapture:
            reopened = reCapture(updated.config)
        return updated, needsRestart
