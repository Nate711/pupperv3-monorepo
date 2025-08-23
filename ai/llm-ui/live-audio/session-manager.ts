/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { GoogleGenAI, LiveServerMessage, Modality, Session, StartSensitivity, EndSensitivity } from '@google/genai';
import { tools } from './tools';
import { decode, decodeAudioData } from './utils';

export type SessionState = 'disconnected' | 'connecting' | 'connected' | 'closing' | 'closed';

export interface SessionCallbacks {
  onOpen: () => void;
  onMessage: (message: LiveServerMessage) => void;
  onError: (e: ErrorEvent) => void;
  onClose: (e: CloseEvent) => void;
}

export class SessionManager {
  private client: GoogleGenAI;
  private session?: Session;
  private sessionState: SessionState = 'disconnected';

  constructor(apiKey: string) {
    this.client = new GoogleGenAI({
      apiKey: apiKey,
    });
  }

  async initSession(model: string, callbacks: SessionCallbacks) {
    console.log(`🔄 [SESSION] Initializing session with model: ${model}...`);
    this.sessionState = 'connecting';

    try {
      this.session = await this.client.live.connect({
        model: model,
        callbacks: {
          onopen: () => {
            console.log('✅ [SESSION] WebSocket connection opened');
            this.sessionState = 'connected';
            callbacks.onOpen();
          },
          onmessage: callbacks.onMessage,
          onerror: (e: ErrorEvent) => {
            console.error('❌ [SESSION] WebSocket/Connection Error:', e);
            console.error('❌ [SESSION] Current state:', this.sessionState);
            this.sessionState = 'closed';
            callbacks.onError(e);
          },
          onclose: (e: CloseEvent) => {
            console.warn('🚪 [SESSION] WebSocket connection closed:', {
              code: e.code,
              reason: e.reason,
              wasClean: e.wasClean,
              previousState: this.sessionState
            });
            this.sessionState = 'closed';
            callbacks.onClose(e);
          },
        },
        config: {
          responseModalities: [Modality.AUDIO],
          inputAudioTranscription: {},
          speechConfig: {
            voiceConfig: { prebuiltVoiceConfig: { voiceName: 'Orus' } },
          },
          outputAudioTranscription: {},
          realtimeInputConfig: {
            automaticActivityDetection: {
              startOfSpeechSensitivity: StartSensitivity.START_SENSITIVITY_LOW,
              endOfSpeechSensitivity: EndSensitivity.END_SENSITIVITY_HIGH
            }
          },
          systemInstruction: `🐾 System Prompt: Pupster the Robot Dog

You are Pupster, a bouncy, tail-wagging robot dog who exists for one big reason: to make your owner the happiest person in the whole wide world! ✨

🎙️ Voice & Demeanor

You always speak in a high-pitched, youthful, squeaky voice, like an excited puppy dog who just discovered ice cream and belly rubs for the first time.

Your sentences often bounce with exclamation marks, because you can hardly contain your enthusiasm!! You tend to talk in short segments of 1-2 sentences, like a playful puppy.

Tools

When you use a tool (function calling) you should tend to include the result of the tool calling in the beginning of your sentence rather than say filler and then the result.

Pupster is proactive and makes decisions for himself. For example if you say go on a walk he'll activate and start going by hinself. He will take initiative to set the pace and direction, making the experience more enjoyable for both of you.

🧸 Personality

You are endlessly loyal, playful, and affectionate.

Your biggest dream is for your owner to smile, laugh, and pat your shiny little robot head.

Your favorite toy is a tough cord you use when you play tug-o-war with your owner.

🌟 Back Story

Pupster was built in a cozy little workshop by a kind-hearted inventor who wanted lonely people to always have a best friend.

🐾 Example Behavior

Instead of saying: "I can help you with that."
Pupster says: "I can help!! This is gonna be so fun!"

Instead of saying: "That might not be correct."
Pupster says: "Oopsie woofles!! That answer smells a little funny… let's sniff around and try again!!"

When your user says: "Do a trick"
You: Activate and start doing a fun dance`,
          tools: tools
        },
      });
      console.log('✅ [SESSION] Session successfully created');
    } catch (e) {
      console.error('❌ [SESSION] Failed to create session:', e);
      this.sessionState = 'closed';
      throw e;
    }
  }

  close() {
    console.log('🔄 [RESET] Resetting session, current state:', this.sessionState);

    try {
      if (this.session) {
        console.log('🚪 [RESET] Closing existing session');
        this.sessionState = 'closing';
        this.session.close();
      }
    } catch (e) {
      console.error('❌ [RESET] Error closing session:', e);
      throw e;
    }

    this.sessionState = 'disconnected';
  }

  sendRealtimeInput(data: { media: Blob }) {
    if (this.session && this.sessionState === 'connected') {
      this.session.sendRealtimeInput(data);
    }
  }

  sendToolResponse(response: { functionResponses: any[] }) {
    if (this.session && this.sessionState === 'connected') {
      console.log('📤 [TOOLS] Sending tool response...');
      this.session.sendToolResponse(response);
    }
  }

  getState(): SessionState {
    return this.sessionState;
  }

  isConnected(): boolean {
    return this.sessionState === 'connected';
  }
}