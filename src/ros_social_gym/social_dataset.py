import os
import json
from torch.utils.data import Dataset, DataLoader
import random

class SocialDataset(Dataset, dataSize):
    def __init__(self, data_path):
        with open(data_path) as json_file:
            self.data_list = json.load(json_file)
            self.data_list.pop(0)
            random.shuffle(self.data_list)
            if (dataSize != 0):
                self.data_list = self.data_list[dataSize:]

    def __len__(self):
        return len(self.data_list)

    def __getitem__(self, idx):
        return self.data_list[idx]
