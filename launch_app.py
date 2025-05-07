from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='audio_tools',
            executable='audio_capture_node',
            name='audio_capture_node',
            output='screen'
        ),
        Node(
            package='simplebotlogic',
            executable='simplebotlogic_node',
            name='simplebotlogic_node',
            output='screen'
        ),
        Node(
            package='tts',
            executable='tts_node',
            name='tts_node',
            output='screen',
            parameters=[
                {
                    'speak_script_path': 'tts-node/src/speak_script.sh',
                    'to_speak_path': 'tts-node/src/tts_input.txt',
                    'model_path': 'models/tts-model.onnx',
                    'model_config_path': 'models/tts-model.onnx.json'
                }
            ]
        ),
        Node(
            package='audio_tools',
            executable='vad_node',
            name='vad_node',
            output='screen'
        ),
        Node(
            package='lowwi',
            executable='lowwi_node',
            name='lowwi',
            output='screen',
            parameters=[
                {
                    'wakeword.phrases': [ "Hey Mycroft" ],
                    'wakeword.models': [ "models/hey_mycroft.onnx" ],
                    'wakeword.min_activations': [ 2 ]
                }
            ] 
        ),
        Node(
            package='simplewhisper',
            executable='simplewhisper_node',
            name='simplewhisper',
            output='screen',
            parameters=[
                {
                    'model_path': 'models/ggml-large-v3-turbo-q5_0.bin',
                    'bot_name': 'LLama',
                    'language': 'nl',
                    'output_topic': '/llm_input',
                    'audio_in_topic': 'whisper/audio_in'
                }
            ]
        ),
        Node(
            package='simplellama',
            executable='simplellama_node',
            name='simplellama',
            output='screen',
            parameters=[
                {
                    'gguf_model_path': 'models/phi-2.Q5_0.gguf',
                    'toggle_topic': '/llm_toggle',
                    'text_output_topic': '/tts_text',
                    'text_input_topic': '/llm_input'
                }
            ]
        )
    ])
