

class RMDControlMode:
    def __init__(self, config):
        self.config = config
        
        self.ROM_SAFE_UPPER = float(self.config["ROM_SAFE_UPPER"])
        self.ROM_SAFE_LOWER = float(self.config["ROM_SAFE_LOWER"])