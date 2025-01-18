#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# pip install openai
from openai import OpenAI
from openai import AssistantEventHandler
from typing_extensions import override
import json

class CentralBrainNode(Node):
    def __init__(self):
        super().__init__('central_brain_node')

        # Create an OpenAI client
        self.client = OpenAI()

        # Create an Assistant that has one "publish_cmd" function tool
        # The cmd argument is an enum we can expand over time.
        # TODO add time series data to the action sequence define. HMM-time depend. 
        # a action x b action not transitive. therefore we need to practice in the simulation. 
        # 1. basic comnbination of actions, from chatgpt imagination and understanding of the action description. D2, D3
        # 2. add time-duration as one variable of each action. D2_0.3, D3_0.5
        # 3. add simulation correction to the action sequence, due to current caos soft structure, like resnet. D2_0.3 + D2_0.1_simulation --KF--> D2_0.2
        # 3.qa1. use ml/world model for the robot to get the correction, use the simulator to train it.
        # 3.1. use ML in simulation to find best time-duration and **combination of actions**.
        # 3.qa2. if we go to 4 without 3 how we can get a reliable combination of actions? we need to close to loop of chatgpt action sequence design.
        # 3.delta. digital-twin, sim-real.
        # 4. action description embeding learning, use abstract words to describe or associte with the action.
        # 1. 2. 3.qa2. 4. first, if not good enough, go to 3.
        #
        # note: 
        # motor_feedback + speech(text) (pending. + visual(function calling)) 
        # -> 4. (context embeding -> search action_sequence -> action sequence gaussian blur (both order and variable))
        # -> 3.qa2. (chatgpt further tune action_sequence -> simulator -> alert (or 3. use ML get action correction term) -> try again or exe action_sequence)
        #
        # after complete 1. 2. and pr/draft, we can decide which one to go next and how to split work.
        self.assistant = self.client.beta.assistants.create(
            name="CentralBrainAssistant",
            instructions=(
                "You are a central brain for a robot. "
                "When the user asks to move or spin, use the 'publish_cmd' function with the correct enum. "
                "Action List:\n" 
                " - D1: Move forward or direction 1\n"
                " - D2: Move forward or direction 2\n"
                " - D3: Move forward or direction 3\n"
                " - CS: Clockwise spin\n"
                " - AS: Anti-clockwise spin\n"
                " - X: Stop action\n"
                "You can expand this set of commands as needed in the future."
            ),
            model="gpt-4o",
            tools=[
                {
                    "type": "function",
                    "function": {
                        "name": "publish_cmd",
                        "description": "Publish a discrete command to cmd_topic",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "cmd": {
                                    "type": "string",
                                    "description": (
                                        "A command to publish. Must be one of the known enumerations. "
                                        "Example: D1, D2, D3, CS, AS, X"
                                    ),
                                    "enum": ["D1", "D2", "D3", "CS", "AS", "X"]
                                }
                            },
                            "required": ["cmd"],
                            "additionalProperties": False
                        },
                        # Uncomment if you want strict JSON validation
                        # "strict": True
                    }
                }
            ]
        )
        
        # Create a new Thread for each query
        # TODO we can create a new thread for each user query if we are using a multi agent sys.
        self.thread = self.client.beta.threads.create()

        # Publisher for discrete robot commands
        self.cmd_publisher = self.create_publisher(String, 'cmd_topic', 10)

        # Publisher for sentences to be spoken (text-to-speech, etc.)
        self.speak_publisher = self.create_publisher(String, 'speak_text', 10)

        # Subscriber to user language queries
        self.query_subscriber = self.create_subscription(
            String,
            'language_query',
            self.query_callback,
            10
        )

        self.get_logger().info("CentralBrainNode has started.")

    def query_callback(self, msg: String):
        """
        Callback for receiving natural language queries from 'language_query' topic.
        We send the user message to the Assistant in a new Thread and try a streaming run.
        """
        user_input = msg.data
        self.get_logger().info(f"Received user query: {user_input}")

        # 2. Add the user's message to the Thread
        self.client.beta.threads.messages.create(
            thread_id=self.thread.id,
            role="user",
            content=user_input
        )

        # 3. Stream the Run so we can capture function calls and partial text
        event_handler = CentralBrainEventHandler(
            node=self,
            thread_id=self.thread.id,
            assistant_id=self.assistant.id
        )

        with self.client.beta.threads.runs.stream(
            thread_id=self.thread.id,
            assistant_id=self.assistant.id,
            event_handler=event_handler
        ) as run_stream:
            # Wait until the run is complete or all function calls are handled
            run_stream.until_done()


class CentralBrainEventHandler(AssistantEventHandler):
    """
    Handles streaming events from the OpenAI Assistants API.
    Specifically, we watch for 'requires_action' events for tool calls,
    and we also capture partial text to publish sentences to speak_text.
    """
    def __init__(self, node: Node, thread_id: str, assistant_id: str):
        super().__init__()
        self.node = node
        self.thread_id = thread_id
        self.assistant_id = assistant_id

        # Buffer for streaming text to detect sentence boundaries
        self.text_buffer = ""
        # Characters that we treat as an end of sentence
        self.eos_chars = {'.', '?', '!', ';'}

    @override
    def on_run_started(self, run):
        """Called when the run is first created."""
        self.current_run = run

    @override
    def on_event(self, event):
        """
        Called for various streaming events. We specifically look for
        'requires_action' -> tool_calls, then respond with tool outputs.
        """
        if event.event == 'thread.run.requires_action':
            self.node.get_logger().info("Model requires action (function call).")
            self.handle_requires_action(event.data, event.data.id)

    def handle_requires_action(self, data, run_id):
        """
        The model wants to call one or more functions. In our scenario,
        that's the 'publish_cmd' function. We parse arguments and publish them.
        """
        required_action = data.required_action
        tool_calls = required_action.submit_tool_outputs.tool_calls

        tool_outputs = []
        for tool_call in tool_calls:
            if tool_call.function.name == "publish_cmd":
                # Parse the arguments (JSON) to retrieve the command
                arguments = json.loads(tool_call.function.arguments)
                cmd = arguments.get("cmd", "")

                # Actually publish the command to cmd_topic
                self.publish_cmd_to_robot(cmd)

                # Provide a response back to the Assistant
                tool_outputs.append({
                    "tool_call_id": tool_call.id,
                    "output": f"Command '{cmd}' published to cmd_topic."
                })

        # Submit any tool outputs so the run can continue/complete
        if tool_outputs:
            self.submit_tool_outputs(tool_outputs, run_id)

    def submit_tool_outputs(self, tool_outputs, run_id):
        """
        Submit function-call outputs (tool outputs) back to the Assistant 
        so it can finalize its response if needed.
        """
        with self.node.client.beta.threads.runs.submit_tool_outputs_stream(
            thread_id=self.thread_id,
            run_id=self.current_run.id,
            tool_outputs=tool_outputs,
            event_handler=AssistantEventHandler(),
        ) as stream:
            # Stream the final piece of text from the Assistant
            for text in stream.text_deltas:
                self.process_stream_text(text)

    @override
    def on_text_created(self, text):
        """
        Called when the Assistant starts output. We can log or partially speak it.
        """
        self.node.get_logger().info("Assistant partial response:\n" + text.value)
        # self.process_stream_text(text.value) # Uncomment to speak partial text

    @override
    def on_text_delta(self, delta, snapshot):
        """
        Called whenever new tokens arrive. We'll parse them for EoS punctuation.
        """
        self.process_stream_text(delta.value)

    def process_stream_text(self, chunk: str):
        """
        Accumulate tokens in self.text_buffer; whenever we see a punctuation
        that marks end-of-sentence, we publish the text to speak_text.
        """
        self.text_buffer += chunk

        # Repeatedly look for any EoS punctuation
        while True:
            found_index = None
            for i, c in enumerate(self.text_buffer):
                if c in self.eos_chars:
                    found_index = i
                    break

            if found_index is None:
                break

            # We found an end-of-sentence punctuation at found_index
            # We'll treat everything up to (and including) that character as a sentence
            sentence = self.text_buffer[: found_index + 1].strip()
            if sentence:
                # Publish it to speak_text
                self.publish_speak_text(sentence)

            # Remove that sentence from the buffer 
            self.text_buffer = self.text_buffer[found_index + 1:].lstrip()

    def publish_speak_text(self, sentence: str):
        """
        Publish the given sentence to the speak_text topic
        (e.g., for TTS or other user feedback).
        """
        msg = String()
        msg.data = sentence
        self.node.speak_publisher.publish(msg)
        self.node.get_logger().info(f"Published speak_text: '{sentence}'")

    def publish_cmd_to_robot(self, cmd: str):
        """
        Publish discrete robot command to 'cmd_topic'.
        """
        msg = String()
        msg.data = cmd
        self.node.cmd_publisher.publish(msg)
        self.node.get_logger().info(f"Published command '{cmd}' to cmd_topic.")


def main(args=None):
    rclpy.init(args=args)
    node = CentralBrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
