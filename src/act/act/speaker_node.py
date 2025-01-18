#!/usr/bin/env python3

import os
from pydub import AudioSegment
from pydub.playback import play

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from gtts import gTTS


def speakText(text, speed_factor=1.25):
    """Converts text to speech, adjusts speed, and plays it."""
    tts = gTTS(text=text, lang='en')
    tmpPath = "tmp/response_audio.mp3"
    tts.save(tmpPath)

    # Load the audio file with pydub
    audio = AudioSegment.from_file(tmpPath)

    # Adjust speed (1.0 = original speed, >1.0 = faster, <1.0 = slower)
    fast_audio = audio._spawn(audio.raw_data, overrides={
        "frame_rate": int(audio.frame_rate * speed_factor)
    }).set_frame_rate(audio.frame_rate)

    # Play the audio
    play(fast_audio)

    # Remove temporary file
    os.remove(tmpPath)


class SpeakerNode(Node):
    """ROS2 Node that subscribes to a topic and speaks out received text."""

    def __init__(self):
        super().__init__('speaker_node')

        # Create a subscriber that listens to the 'speak_text' topic.
        # We expect std_msgs/msg/String messages on this topic.
        self.subscription = self.create_subscription(
            String,
            'speak_text',
            self.listener_callback,
            10  # QoS History depth
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info("SpeakerNode has been started. Waiting for messages...")

    def listener_callback(self, msg):
        """Callback whenever a new message arrives on 'speak_text'."""
        received_text = msg.data
        self.get_logger().info(f"Received text to speak: {received_text}")
        
        # Convert text to speech and play audio with increased speed
        speakText(received_text)


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
