from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='audio_tools',
            executable='audio_playback_node',
            name='audio_playback_node',
            output='screen',
            parameters=[
            {
              'audio_topic': '/tts_samples'
            }
            ]
        ),
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
            output='screen',
            parameters =[
            {
               'bot_name': 'Hey Tom'
            }
            ]
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
                    'wakeword.phrases': [ "Hey Tom" ],
                    'wakeword.models': [ "models/hey_tom.onnx" ],
                    'wakeword.min_activations': [ 2 ]
                }
            ] 
        ),
        # Node(
        #     package='audio_tools',
        #     executable='audio_playback_node',
        #     name='audio_playback_node',
        #     output='screen',
        #     parameters=[
        #     {
        #         'audio_topic': 'whisper/audio_in'
        #     }
        #     ]
        # ),
        Node(
            package='simplewhisper',
            executable='simplewhisper_node',
            name='simplewhisper',
            output='screen',
            parameters=[
                {
                    'model_path': 'models/ggml-medium-q5_0.bin',
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
                    'gguf_model_path': 'models/fietje-2b-chat-q5_k_m.gguf',
                    'toggle_topic': '/llm_toggle',
                    'text_output_topic': '/tts_text',
                    'text_input_topic': '/llm_input'
                }
            ]
        )
    ])
