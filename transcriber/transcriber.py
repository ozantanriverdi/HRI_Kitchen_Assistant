import torch
from transformers import pipeline
from transformers.utils import is_flash_attn_2_available
import time
import threading
import json
class Transcriber:
    def __init__(self, micRecorder):
        self.micrecorder = micRecorder
        self.setup()
        self.output = {}
        self.filePath = None
        self.index = 0

    def setup(self):
        self.pipe = pipeline(
            "automatic-speech-recognition",
            model="openai/whisper-large-v3",
            torch_dtype=torch.float16,
            device="cuda:0",
            model_kwargs={"attn_implementation": "flash_attention_2"} if is_flash_attn_2_available() else {"attn_implementation": "sdpa"},
        )
        self.micrecorder.shouldListen=True

    def transcribeAudio(self):
        text = self.pipe(
        self.filePath,
        chunk_length_s=30,
        batch_size=2,
        return_timestamps=False,
    )['text']
        self.output = {"text" : text, "index" : self.index}
        self.index += 1
        torch.cuda.empty_cache()