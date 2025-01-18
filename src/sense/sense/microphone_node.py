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
        self.audio_index = 1  # Counter for naming audio files

        # --- Accumulation Buffers (for VAD mode) ---
        self.accumulated_audio_arrays = []
        self.accumulated_length = 0.0  # in seconds
        self.sample_rate = 16000

        # VAD Setup
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(1)

        # Queues for background transcription
        self.transcription_queue = queue.Queue()
        self.transcription_results = queue.Queue()

        # Start background thread for transcribing audio
        self.transcription_thread = threading.Thread(
            target=self.transcription_worker, daemon=True
        )
        self.transcription_thread.start()

        # Audio device & mode
        self.input_device = None
        self.recording_mode = None

        # Space-mode additional attributes
        self.space_recording_active = False
        self.space_mode_stream = None
        self.space_mode_buffer = []  # list of float32 numpy arrays
        self.space_mode_thread = None

        self.get_logger().info("Audio Recorder Node has started.")
        self.select_audio_device()
        self.choose_recording_mode()

        # If user chose 'space' mode, start a non-blocking thread to handle ENTER presses
        if self.recording_mode == 'space':
            self.space_mode_thread = threading.Thread(
                target=self.space_mode_key_listener,
                daemon=True
            )
            self.space_mode_thread.start()

        self.run()

    # ----------------------------------------------------------------------
    #                    DEVICE SELECTION & MODE
    # ----------------------------------------------------------------------
    def list_audio_devices(self):
        devices = sd.query_devices()
        for i, dev in enumerate(devices):
            self.get_logger().info(f"Device {i}: {dev['name']}")

    def select_audio_device(self):
        self.list_audio_devices()
        while True:
            user_in = input("Enter the device index for recording: ")
            try:
                dev_index = int(user_in)
                sd.check_input_settings(device=dev_index, samplerate=self.sample_rate)
                self.input_device = dev_index
                self.get_logger().info(f"Selected device index: {dev_index}")
                break
            except Exception as e:
                self.get_logger().error(f"Invalid device index: {e}. Try again.")

    def choose_recording_mode(self):
        while True:
            mode = input("Choose recording mode ('vad' or 'space'): ").strip().lower()
            if mode in ['vad', 'space']:
                self.recording_mode = mode
                self.get_logger().info(f"Selected recording mode: {mode}")
                break
            else:
                self.get_logger().error("Invalid choice. Please enter 'vad' or 'space'.")

    # ----------------------------------------------------------------------
    #                    VAD MODE RECORDING
    # ----------------------------------------------------------------------
    def record_audio_vad(self, filename, fs=16000):
        """
        Records audio until silence is detected (via VAD).
        """
        def callback(indata, frames, time_info, status):
            nonlocal recordingActive
            if status:
                self.get_logger().warning(f"Stream status: {status}")

            data_int16 = (indata * 32767).astype(np.int16)
            if len(data_int16) != 320:  # 20ms @ 16kHz
                return

            is_speech = self.vad.is_speech(data_int16.tobytes(), fs)
            if is_speech:
                recording.append(indata.copy())  # keep float32
                recordingActive = True
            else:
                if recordingActive:
                    recordingActive = False
                    self.get_logger().info("Silence detected, stopping recording.")

        recordingActive = False
        recording = []
        self.get_logger().info("VAD Mode: Recording... (Speak, then be silent to stop)")

        with sd.InputStream(callback=callback,
                            dtype='float32',
                            channels=1,
                            samplerate=fs,
                            blocksize=320,
                            device=self.input_device):
            # Wait until speech is actually detected => callback stops upon silence
            while not recordingActive:
                time.sleep(0.2)

        if recording:
            recording_array = np.concatenate(recording, axis=0)
            # Normalize and convert
            recording_int = np.int16(
                recording_array / np.max(np.abs(recording_array)) * 32767
            )
            wavWrite(filename, fs, recording_int)
            self.get_logger().info(f"Audio written to {filename}")
        else:
            self.get_logger().warning("No audio data recorded.")

    # ----------------------------------------------------------------------
    #        SPACE MODE (Manual) - Non-blocking Implementation
    # ----------------------------------------------------------------------
    def space_mode_callback(self, indata, frames, time_info, status):
        """Callback used while we're actively recording in 'space' mode."""
        if status:
            self.get_logger().warning(f"Stream status: {status}")
        self.space_mode_buffer.append(indata.copy())

    def space_mode_key_listener(self):
        """
        Runs in a separate thread.
        Each time the user presses ENTER, we toggle the recording state.
        """
        self.get_logger().info(
            "Manual Mode (non-blocking): Press ENTER to start/stop recording at any time..."
        )

        while True:
            input()  # This blocks ONLY this thread, not the main ROS thread

            if not self.space_recording_active:
                # --- Start Recording ---
                self.space_recording_active = True
                self.space_mode_buffer = []
                self.space_mode_stream = sd.InputStream(
                    callback=self.space_mode_callback,
                    dtype='float32',
                    channels=1,
                    samplerate=self.sample_rate,
                    device=self.input_device
                )
                self.space_mode_stream.start()
                self.get_logger().info("Manual recording started.")
            else:
                # --- Stop Recording ---
                self.space_mode_stream.stop()
                self.space_mode_stream.close()
                self.space_recording_active = False
                self.get_logger().info("Manual recording stopped.")

                # Convert buffer to .wav
                if self.space_mode_buffer:
                    filename = f"tmp/audio_{self.audio_index}.wav"
                    recording_array = np.concatenate(self.space_mode_buffer, axis=0)
                    recording_int = np.int16(
                        recording_array / np.max(np.abs(recording_array)) * 32767
                    )
                    wavWrite(filename, self.sample_rate, recording_int)
                    self.get_logger().info(f"[space] Audio saved to {filename}")

                    # Enqueue for transcription
                    self.transcription_queue.put(filename)
                    self.get_logger().info(f"Enqueued {filename} for transcription.")

                    # Move to next index for the next file
                    self.audio_index += 1

                else:
                    self.get_logger().warning("No audio data was recorded.")

    # ----------------------------------------------------------------------
    #                     TRANSCRIPTION (OpenAI Whisper)
    # ----------------------------------------------------------------------
    def transcribe_audio(self, audio_file_path):
        """Blocks while calling the OpenAI Whisper API to transcribe the audio."""
        try:
            with open(audio_file_path, "rb") as audio_file:
                # NOTE: The actual openai API call may differ (depending on library version).
                # Adjust as needed (openai.Audio, openai.Audio.transcribe, etc.).
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
        """Worker thread for pulling audio files and transcribing them."""
        while True:
            audio_file_path = self.transcription_queue.get()
            if audio_file_path is None:
                break  # Stop signal

            text = self.transcribe_audio(audio_file_path)
            
            if os.path.exists(audio_file_path):
                os.remove(audio_file_path)
                
            self.transcription_results.put(text)

    def process_transcription_results(self, accumulated_text):
        """
        Fetch new transcriptions from the queue and publish:
         - 'space' mode: publish each chunk in full, immediately
         - 'vad'   mode: accumulate into a conversation, only last 20 words
        """
        while not self.transcription_results.empty():
            new_text = self.transcription_results.get()
            if not new_text:
                continue

            if self.recording_mode == 'space':
                # Publish entire chunk
                message = new_text.strip()
                if message:
                    msg = String()
                    msg.data = message
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"[space] Published transcription:\n{message}")
            else:
                # VAD mode: accumulate into a growing conversation
                accumulated_text += new_text.split()
                # Publish only the last 20 words
                message = " ".join(accumulated_text[-20:])
                if message.strip():
                    msg = String()
                    msg.data = message
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"[vad] Published transcription (last 20 words):\n{message}")

    # ----------------------------------------------------------------------
    #                            MAIN LOOP
    # ----------------------------------------------------------------------
    def run(self):
        """
        Main loop for the node:
          - If 'vad', repeatedly record short segments and accumulate.
          - If 'space', do nothing special here (because it's non-blocking).
            The user triggers recordings in a separate thread via ENTER.
          - Always process transcription results frequently.
        """
        if self.recording_mode == 'vad':
            # VAD loop: record, accumulate, once ~5+ seconds is reached -> transcribe
            while rclpy.ok():
                audio_file = f"tmp/audio_{self.audio_index}.wav"
                self.get_logger().info(f"VAD: Starting new recording session => {audio_file}")
                self.record_audio_vad(audio_file)

                # Read the small wav snippet
                try:
                    sr, data = wavRead(audio_file)
                    length_in_seconds = len(data) / sr
                except Exception as e:
                    self.get_logger().error(f"Error reading {audio_file}: {e}")
                    if os.path.exists(audio_file):
                        os.remove(audio_file)
                    continue

                # Accumulate
                self.accumulated_audio_arrays.append(data)
                self.accumulated_length += length_in_seconds
                # Remove short snippet after we've stored the data
                os.remove(audio_file)

                # If >= 5s, combine and enqueue
                if self.accumulated_length >= 5.0:
                    combined_data = np.concatenate(self.accumulated_audio_arrays, axis=0)
                    os.makedirs(os.path.dirname(combined_file), exist_ok=True)
                    wavWrite(combined_file, self.sample_rate, combined_data)
                    self.get_logger().info(
                        f"Accumulated {self.accumulated_length:.2f} s of audio. "
                        f"Enqueuing {combined_file} for transcription..."
                    )
                    self.transcription_queue.put(combined_file)

                    # Reset
                    self.accumulated_audio_arrays.clear()
                    self.accumulated_length = 0.0

                # Process any completed transcriptions
                self.process_transcription_results(accumulated_text)
                self.audio_index += 1

                self.get_logger().info(
                    "\n(VAD) Press Ctrl+C to exit at any time, or continue speaking...\n"
                )
        else:
            # 'space' MODE: We do not block for recordings here
            self.get_logger().info("Non-blocking manual mode loop started.")
            self.get_logger().info(
                "Press ENTER in the console to start/stop recording. "
                "This main loop will keep spinning, processing transcripts."
            )
            # Just keep spinning until node is shut down
            while rclpy.ok():
                self.process_transcription_results(accumulated_text)
                time.sleep(0.2)  # or use a Timer, just to reduce CPU usage

    # ----------------------------------------------------------------------
    #                         MAIN ENTRY POINT
    # ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = AudioRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nExiting program. Goodbye!")
    finally:
        # Signal transcription worker to stop
        node.transcription_queue.put(None)
        node.transcription_thread.join()
        node.destroy_node()

        # Clean up leftover combined audio (VAD) if it exists
        if os.path.exists(combined_file):
            os.remove(combined_file)
            node.get_logger().info(f"Removed combined audio file: {combined_file}")

        rclpy.shutdown()

if __name__ == "__main__":
    main()
