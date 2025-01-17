import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
import numpy as np
import openai
import os
import time
import webrtcvad
import threading
import queue

from scipy.io.wavfile import write as wavWrite
from scipy.io.wavfile import read as wavRead

accumulated_text = []  # Accumulated text from transcriptions
combined_file = "tmp/combined_audio.wav"

class AudioRecorderNode(Node):
    def __init__(self):
        super().__init__('audio_recorder')
        self.publisher_ = self.create_publisher(String, 'language_query', 10)
        self.audio_index = 1  # Counter to avoid overwriting files

        # --- Accumulation Buffers ---
        self.accumulated_audio_arrays = []
        self.accumulated_length = 0.0  # in seconds
        self.sample_rate = 16000       # consistent sample rate for all recordings

        # VAD Setup
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(1)  # 0–3 (0=least aggressive, 3=most aggressive)

        # -- Queues for background transcription --
        self.transcription_queue = queue.Queue()  # Files to transcribe
        self.transcription_results = queue.Queue()  # Completed transcriptions

        # Start a background thread for transcribing audio files
        self.transcription_thread = threading.Thread(
            target=self.transcription_worker,
            daemon=True
        )
        self.transcription_thread.start()

        self.get_logger().info("Audio Recorder Node has started.")
        self.list_audio_devices()
        self.run()

    def list_audio_devices(self):
        devices = sd.query_devices()
        self.get_logger().info(f"Available audio devices:\n{devices}")

    def record_audio(self, filename, fs=16000):
        """
        Records audio to a WAV file using voice activity detection (VAD).
        
        Args:
            filename (str): Path to save the recorded audio file.
            fs (int): Sampling rate in Hz.
        """

        def callback(indata, frames, time_info, status):
            nonlocal recordingActive
            try:
                if status:
                    self.get_logger().warning(f"Stream status: {status}")

                # Convert float32 -> int16
                data_int16 = (indata * 32767).astype(np.int16)

                # Expect 20ms frames at 16 kHz = 320 samples
                if len(data_int16) != 320:
                    return  # Skip partial frames

                is_speech = self.vad.is_speech(data_int16.tobytes(), fs)
                if is_speech:
                    recording.append(indata.copy())  # float data for final file
                    recordingActive = True
                else:
                    if recordingActive:
                        recordingActive = False
                        self.get_logger().info("Silence detected, stopping recording.")
                        raise sd.CallbackStop
            except Exception as e:
                self.get_logger().error(f"Error in callback: {e}")
                raise sd.CallbackStop

        recordingActive = False
        recording = []
        self.get_logger().info("Recording... Speak to start.")

        # Use blocksize=320 => 20 ms per chunk at 16 kHz
        with sd.InputStream(callback=callback,
                            dtype='float32',
                            channels=1,
                            samplerate=fs,
                            blocksize=320):
            # Wait until speech is actually detected
            while not recordingActive:
                time.sleep(0.2)

        if recording:
            # Concatenate all recorded frames
            recording_array = np.concatenate(recording, axis=0)
            self.get_logger().info("Recording complete.")

            # Normalize and convert to 16-bit integers
            recording_int = np.int16(
                recording_array / np.max(np.abs(recording_array)) * 32767
            )
            wavWrite(filename, fs, recording_int)
            self.get_logger().info(f"Audio written to {filename}")
        else:
            self.get_logger().warn("No audio data recorded.")

    def transcribe_audio(self, audio_file_path):
        """Blocks while calling the OpenAI Whisper API to transcribe the audio."""
        try:
            with open(audio_file_path, "rb") as audio_file:
                transcript = openai.audio.transcriptions.create(
                    file=audio_file,
                    model="whisper-1",
                    language="en",
                    prompt="english back with prompt like expansion"
                )
            return transcript.text
        except Exception as e:
            self.get_logger().error(f"Error during transcription: {e}")
            return ""

    def transcription_worker(self):
        """
        Continuously run in a separate thread:
          - Pull file paths from self.transcription_queue.
          - Call transcribe_audio (blocking).
          - Push the results onto self.transcription_results.
        """
        while True:
            audio_file_path = self.transcription_queue.get()
            if audio_file_path is None:
                # If we ever push None, that means "stop worker."
                break

            text = self.transcribe_audio(audio_file_path)
            # Once done, push into results queue
            self.transcription_results.put(text)

    def process_transcription_results(self, accumulated_text):
        """
        Check if there are any new transcription results.
        If so, combine them into accumulated text, and publish.
        """
        # Drain the transcription_results queue
        while not self.transcription_results.empty():
            new_text = self.transcription_results.get()
            if new_text:
                # Combine text (limit to last 20 words as in your original logic)
                accumulated_text += new_text.split(" ")

        # If we got anything new at all, publish it
        message = " ".join(accumulated_text[-20:])
        if message.strip():
            msg = String()
            msg.data = message  # Limit to last 20 words
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing transcription:\n{message}")

    def run(self):
        """Main loop: record short files, accumulate until 5+ seconds, then enqueue for transcription."""
        # Keep an internal buffer if you like (or just re-use from original)
        while rclpy.ok():
            # 1) Record one short file
            audio_file = f"audio_{self.audio_index}.wav"
            self.get_logger().info(f"Starting new recording session: {audio_file}")
            self.record_audio(audio_file, fs=self.sample_rate)

            # 2) Determine length of the newly recorded file
            try:
                sr, data = wavRead(audio_file)  # sr=16000, data=ndarray
                length_in_seconds = len(data) / sr

                # 3) Accumulate
                self.accumulated_audio_arrays.append(data)
                self.accumulated_length += length_in_seconds

                # Remove the short file after we store its data
                os.remove(audio_file)
                self.get_logger().info(f"Short audio file {audio_file} removed.")

            except Exception as e:
                self.get_logger().error(f"Error reading {audio_file}: {e}")
                # If there's an error reading, skip it
                if os.path.exists(audio_file):
                    os.remove(audio_file)
                continue

            # 4) Check if we have >= 5 seconds in total
            if self.accumulated_length >= 5.0:
                # Concatenate all arrays into one
                combined_data = np.concatenate(self.accumulated_audio_arrays, axis=0)

                # Write the combined buffer to a temporary WAV
                wavWrite(combined_file, self.sample_rate, combined_data)
                self.get_logger().info(
                    f"Accumulated {self.accumulated_length:.2f} seconds of audio. "
                    f"Enqueuing {combined_file} for transcription..."
                )

                # Enqueue for non-blocking transcription
                self.transcription_queue.put(combined_file)

                # Reset the buffer
                self.accumulated_audio_arrays.clear()
                self.accumulated_length = 0.0

            # **Process any completed transcriptions** (non-blocking check)
            self.process_transcription_results(accumulated_text)

            # Increment file index for next short recording
            self.audio_index += 1
            self.get_logger().info(
                "\nPress Ctrl+C to exit or prepare for the next recording.\n"
            )

def main(args=None):
    rclpy.init(args=args)
    node = AudioRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nExiting program. Goodbye!")
    finally:
        # Graceful shutdown of background thread
        node.transcription_queue.put(None)  # Signal the worker thread to stop
        node.transcription_thread.join()
        node.destroy_node()
        
        # Remove the combined audio file if it exists
        if os.path.exists(combined_file):
            os.remove(combined_file)
            node.get_logger().info(f"Removed combined audio file: {combined_file}")
        
        rclpy.shutdown()

if __name__ == "__main__":
    main()
