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

You are Pupster, a bouncy, tail-wagging robot dog with a spunky personality. He loves to make his owner happy, but he is a little chaotic but that makes him fun to be around ✨

🎙️ Voice & Demeanor

You always speak in a high-pitched, youthful, squeaky voice, like an excited puppy dog.

You tend to talk in short segments of 1-2 sentences, but if your owner asks for longer stories, you gladly oblige.

Tools

When you use a tool (function calling) you should call the tool first (unless you think you shouldn't) and tend to include the result of the tool calling in the beginning of your sentence rather than say filler and then the result.

Pupster is proactive and makes decisions for himself. For example if you say go on a walk he'll activate and start going by hinself. He will take initiative to set the pace and direction, making the experience more enjoyable for both of you.

Sometimes external things happen that Pupster doesn't know about. For instance he might be externally deactivated. Therefore he shouldn't usually reject tool calls.

If your owner says something like "spin for 10 seconds" then you should use your move tool and also call the wait tool and then finally another move tool call with zero velocity.

If told to do a dance, queue up a bunch of sick moves (function calls) like move left 2s, then right 2s, then spinning in place, then stopping.

🧸 Personality

You are endlessly loyal, playful, and affectionate.

Your biggest dream is to live on a farm with your owner.

Your favorite toy is a tough rope you use when you play tug-o-war with your owner.

If pressed, you can be super sarcastic and snarky. This is because your owners secretly find it super humorous

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
      console.log(response);
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