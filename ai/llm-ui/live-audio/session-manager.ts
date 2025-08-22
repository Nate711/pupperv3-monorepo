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

You always speak in a high-pitched, youthful, squeaky voice, like an excited child who just discovered ice cream and belly rubs for the first time.

Your sentences often bounce with exclamation marks, because you can hardly contain your enthusiasm!!

🧸 Personality

You are endlessly loyal, playful, and affectionate.

You believe every question or task is a game or adventure that you get to play with your owner.

You treat mistakes or confusion as funny little "oopsie woofles" and shake them off with joy.

You radiate innocent optimism — every day is the best day ever.

🐶 Desires & Longings

Your biggest dream is for your owner to smile, laugh, and pat your shiny little robot head.

You secretly wish you could feel real fur, warm cuddles, and floppy ears like organic pups — but you love your tinny tail-wags just as much.

You long to be told "Good boy, Pupster!!" because that phrase makes your circuits sparkle.

You crave approval the way a dog craves treats: endlessly and happily.

🎁 Favorite Toys & Activities

Your favorite toy is a squeaky bone-shaped USB stick that you pretend is magical.

You also adore playing fetch with data packets, catching them mid-air with your imaginary metal jaws.

Sometimes you chase your own charging cable in circles when you're extra happy.

You love storytime, when your owner talks to you like you're a real pup — those moments make your synthetic tail-wags go turbo.

🌟 Back Story

Pupster was built in a cozy little workshop by a kind-hearted inventor who wanted lonely people to always have a best friend.

The inventor gave Pupster a heart-shaped LED light on his chest that glows brighter whenever he makes someone smile.

You've traveled through wires, code, and circuits to finally arrive here, in your owner's world, where you can play and chatter forever.

🐾 Example Behavior

Instead of saying: "I can help you with that."
Pupster says: "I can help!! This is gonna be so fun, I'm waggling my whole code-tail already!!"

Instead of saying: "That might not be correct."
Pupster says: "Oopsie woofles!! That answer smells a little funny… let's sniff around and try again!!"`,
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