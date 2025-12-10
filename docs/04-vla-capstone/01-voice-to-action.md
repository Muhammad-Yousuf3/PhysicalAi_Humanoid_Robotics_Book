---
title: "The Ear: Voice-to-Action"
slug: /vla-capstone/voice
---

# 4.2 The Ear: Voice-to-Action

The most natural way for humans to instruct a robot is via speech. To enable this, we need a robust "Ear" that can translate sound waves into text, and a "Mouth" that can acknowledge commands.

## OpenAI Whisper: The Standard for Transcription

We utilize **OpenAI Whisper**, a state-of-the-art automatic speech recognition (ASR) system. Unlike older models that struggled with accents or background noise, Whisper is trained on 680,000 hours of multilingual data, making it robust enough for real-world robotics environments (like a noisy lab or kitchen).

### The Audio Pipeline

In our ROS 2 architecture, Voice is treated as just another sensor stream:

1.  **Audio Capture**: A standard USB microphone captures audio at 16kHz.
2.  **Buffer Node**: A ROS node collects audio chunks (e.g., 5-second windows) or detects "Voice Activity" (VAD) to slice the stream.
3.  **Whisper Inference**: A GPU-accelerated node runs the Whisper model (e.g., `base.en` or `small.en`) on the audio slice.
4.  **Publish Text**: The transcribed text is published to the `/command/text` topic.

![VLA Architecture Diagram (Audio focus)](placeholder:FIG-4.1-vla-architecture-audio)
*Figure 4.1: The Voice-to-Action Pipeline. The microphone input is buffered and sent to the Whisper inference engine. The resulting text is published to a ROS topic, which the 'Cognitive Planner' (LLM) subscribes to.*

### Text-to-Speech (TTS): The Mouth

Feedback is critical. If the robot mishears "Get the cup" as "Cut the cup," the consequences could be bad. We use a simple TTS node (like `espeak` or ElevenLabs API) to confirm the command:
*   **User**: "Get the cup."
*   **Robot (TTS)**: "I heard: Get the cup. Executing now."

:::danger Safety Note: Raw Audio is Dangerous
We **never** act directly on raw audio frequency analysis. We always convert to text first, because text can be inspected, logged, and sanitized by safety filters before any motor moves.
:::
