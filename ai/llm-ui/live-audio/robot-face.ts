/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import {LitElement, css, html} from 'lit';
import {customElement, property, state} from 'lit/decorators.js';
import {Analyser} from './analyser';
import './spectrogram';

@customElement('gdm-robot-face')
export class GdmRobotFace extends LitElement {
  private inputAnalyser!: Analyser;
  private outputAnalyser!: Analyser;
  
  @state() private currentState: 'idle' | 'listening' | 'thinking' | 'speaking' | 'muted' = 'idle';
  @state() private blinkClass = '';
  @state() private pingClass = '';
  @state() private gazeX = 0;
  @state() private gazeY = 0;

  private _outputNode!: AudioNode;
  private _inputNode!: AudioNode;
  private animationFrameId!: number;
  private blinkTimer!: number;
  private speakingAnimationFrameId!: number;

  @property()
  set outputNode(node: AudioNode) {
    this._outputNode = node;
    this.outputAnalyser = new Analyser(this._outputNode);
  }

  get outputNode() {
    return this._outputNode;
  }

  @property()
  set inputNode(node: AudioNode) {
    this._inputNode = node;
    this.inputAnalyser = new Analyser(this._inputNode);
  }

  get inputNode() {
    return this._inputNode;
  }

  static styles = css`
    :host {
      display: block;
      width: 100%;
      height: 100%;
      position: absolute;
      inset: 0;
    }

    .robot-face-container {
      width: 100%;
      height: 100%;
      display: flex;
      align-items: center;
      justify-content: center;
      background: #0a0a0a;
    }

    .stage {
      position: relative;
      background: #000;
      border-radius: 20px;
      box-shadow: 0 10px 40px rgba(0, 0, 0, .6), inset 0 0 40px rgba(0, 120, 255, .08);
      padding: 24px;
      max-width: min(980px, 94vw);
      width: 100%;
    }

    :root {
      --ring-dark: #2a2f36;
      --iris-blue: #19a2e6;
      --iris-blue-2: #2ec4ff;
      --iris-core: #0b1727;
      --pupil: #060d13;
      --iris-h: 200;
      --iris-s: 90;
      --iris-l: 52;
      --gaze-x: 0px;
      --gaze-y: 0px;
      --glow-alpha: .18;
      --dim: 1;
    }

    svg {
      display: block;
      width: 100%;
      height: auto;
      filter: saturate(var(--dim));
    }

    /* States */
    .stage[data-state="idle"] {
      --iris-s: 90;
      --glow-alpha: .12;
    }

    .stage[data-state="listening"] {
      --iris-s: 98;
      --glow-alpha: .20;
    }

    .stage[data-state="thinking"] {
      --iris-s: 65;
      --glow-alpha: 0;
    }

    .stage[data-state="speaking"] {
      --iris-s: 100;
      --glow-alpha: .32;
    }

    .stage[data-state="muted"] {
      --iris-s: 0;
      --glow-alpha: 0;
      --dim: .7;
      filter: grayscale(.2) brightness(.85);
    }

    /* Halos */
    .halo {
      opacity: var(--glow-alpha);
      transform-origin: center;
    }

    .stage[data-state="idle"] .halo {
      animation: breathe 7s ease-in-out infinite;
    }

    .stage[data-state="listening"] .halo {
      animation: pulse 900ms ease-in-out infinite;
    }

    .stage[data-state="speaking"] .halo {
      animation: throb 320ms ease-in-out infinite;
    }

    @keyframes breathe {
      0%, 100% { transform: scale(1); }
      50% { transform: scale(1.02); }
    }

    @keyframes pulse {
      0%, 100% { transform: scale(1); }
      50% { transform: scale(1.07); }
    }

    @keyframes throb {
      0%, 100% { transform: scale(1); }
      50% { transform: scale(1.06); }
    }

    /* Gaze */
    .pupil, .glints {
      transform: translate(var(--gaze-x), var(--gaze-y));
    }

    /* Eyelids */
    .lidTop, .lidBot {
      transform: translateY(0);
    }

    /* Blink animation */
    .blink .lidTop, .blink .lidBot {
      animation: blinkOnce 230ms cubic-bezier(.3, .7, .2, 1) 1;
    }

    @keyframes blinkOnce {
      0% { transform: translateY(0); }
      45% { transform: translateY(140px); }
      55% { transform: translateY(140px); }
      100% { transform: translateY(0); }
    }

    /* Listening ping */
    .ping .ripple {
      animation: ripple 700ms ease-out 1;
    }

    @keyframes ripple {
      0% { transform: scale(.2); opacity: .35; }
      100% { transform: scale(1.6); opacity: 0; }
    }

    /* Thinking dots */
    .dots {
      opacity: 0;
    }

    .stage[data-state="thinking"] .dots {
      opacity: 1;
    }

    .stage[data-state="thinking"] .dots circle {
      animation: dot 900ms ease-in-out infinite;
    }

    .stage[data-state="thinking"] .dots circle:nth-child(2) {
      animation-delay: .12s;
    }

    .stage[data-state="thinking"] .dots circle:nth-child(3) {
      animation-delay: .24s;
    }

    @keyframes dot {
      0%, 100% { opacity: .25; transform: translateY(0); }
      50% { opacity: 1; transform: translateY(-5px); }
    }

    /* Equalizer */
    .eq {
      opacity: 0;
      transform-origin: 50% 100%;
    }

    .stage[data-state="speaking"] .eq {
      opacity: 1;
    }

    .eq rect {
      transform-origin: 50% 100%;
    }

    @media (prefers-reduced-motion: reduce) {
      .halo, .dots circle {
        animation: none !important;
      }
    }
  `;

  connectedCallback() {
    super.connectedCallback();
    this.startAnimation();
    this.scheduleIdleBlink();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this.stopAnimation();
    if (this.blinkTimer) {
      clearTimeout(this.blinkTimer);
    }
  }

  private startAnimation() {
    const animate = () => {
      if (this.inputAnalyser && this.outputAnalyser) {
        this.inputAnalyser.update();
        this.outputAnalyser.update();

        // Determine state based on audio levels
        const inputLevel = this.inputAnalyser.data[0] / 255;
        const outputLevel = this.outputAnalyser.data[0] / 255;
        const inputAverage = this.inputAnalyser.data.slice(0, 10).reduce((a, b) => a + b, 0) / (10 * 255);

        if (outputLevel > 0.05) {
          this.setState('speaking');
          this.updateSpeakingAnimation();
        } else if (inputLevel > 0.02 || inputAverage > 0.01) {
          this.setState('listening');
          this.updateListeningGaze();
        } else {
          this.setState('idle');
        }
      }

      this.animationFrameId = requestAnimationFrame(animate);
    };
    
    this.animationFrameId = requestAnimationFrame(animate);
  }

  private stopAnimation() {
    if (this.animationFrameId) {
      cancelAnimationFrame(this.animationFrameId);
    }
    if (this.speakingAnimationFrameId) {
      cancelAnimationFrame(this.speakingAnimationFrameId);
    }
  }

  private setState(newState: typeof this.currentState) {
    if (this.currentState !== newState) {
      this.currentState = newState;
      this.requestUpdate();
    }
  }

  private updateSpeakingAnimation() {
    if (this.speakingAnimationFrameId) {
      cancelAnimationFrame(this.speakingAnimationFrameId);
    }

    const animate = () => {
      const stage = this.shadowRoot?.querySelector('.stage') as HTMLElement;
      const eqBars = stage?.querySelectorAll('.eq rect') as NodeListOf<SVGRectElement>;
      const brows = stage?.querySelectorAll('.brow') as NodeListOf<SVGPathElement>;

      if (eqBars && this.outputAnalyser) {
        const energy = Math.pow(this.outputAnalyser.data[0] / 255, 2);
        
        eqBars.forEach((bar, i) => {
          const jitter = Math.max(0.08, energy + (Math.random() - .5) * 0.15);
          const h = 100 * jitter * (1 + (i === 2 ? 0.35 : 0));
          bar.setAttribute('height', h.toFixed(1));
          bar.setAttribute('y', (-h).toFixed(1));
        });

        if (brows) {
          const browY = -2 - 4 * energy;
          brows.forEach(b => b.setAttribute('transform', `translate(0, ${browY.toFixed(2)})`));
        }
      }

      if (this.currentState === 'speaking') {
        this.speakingAnimationFrameId = requestAnimationFrame(animate);
      }
    };

    this.speakingAnimationFrameId = requestAnimationFrame(animate);
  }

  private updateListeningGaze() {
    // Enhanced gaze movement and effects during listening
    if (this.currentState === 'listening') {
      const time = performance.now() / 1000;
      const inputLevel = this.inputAnalyser.data[0] / 255;
      
      // More dynamic gaze movement based on input level
      const intensity = 1 + inputLevel * 2;
      this.gazeX = Math.sin(time * 2.1 * intensity) * (8 + inputLevel * 10);
      this.gazeY = Math.sin(time * 4.2 * intensity) * (5 + inputLevel * 6);
      
      this.style.setProperty('--gaze-x', `${this.gazeX}px`);
      this.style.setProperty('--gaze-y', `${this.gazeY}px`);
      
      // Trigger ping effect on strong input
      if (inputLevel > 0.3) {
        this.triggerListeningPing();
      }
    }
  }

  private triggerListeningPing() {
    this.pingClass = 'ping';
    this.requestUpdate();
    
    setTimeout(() => {
      this.pingClass = '';
      this.requestUpdate();
    }, 750);
  }

  private scheduleIdleBlink() {
    if (this.blinkTimer) {
      clearTimeout(this.blinkTimer);
    }
    
    if (this.currentState === 'idle') {
      const wait = 1800 + Math.random() * 4200;
      this.blinkTimer = setTimeout(() => {
        this.blink();
        this.scheduleIdleBlink();
      }, wait);
    }
  }

  private blink() {
    this.blinkClass = 'blink';
    this.requestUpdate();
    
    setTimeout(() => {
      this.blinkClass = '';
      this.requestUpdate();
    }, 260);
  }

  protected render() {
    return html`
      <div class="robot-face-container">
        <gdm-spectrogram 
          .audioNode=${this._inputNode}
          position="left"
          label="MIC INPUT"
          width="256" 
          height="128">
        </gdm-spectrogram>
        <gdm-spectrogram 
          .audioNode=${this._outputNode}
          position="right"
          label="AI OUTPUT"
          width="256" 
          height="128">
        </gdm-spectrogram>
        <div class="stage ${this.blinkClass} ${this.pingClass}" data-state="${this.currentState}">
          <svg viewBox="0 0 900 420" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">
            <defs>
              <filter id="blur8">
                <feGaussianBlur stdDeviation="8" />
              </filter>
              <radialGradient id="gIris" cx="35%" cy="35%" r="80%">
                <stop offset="0%" stop-color="#0f2438" />
                <stop offset="70%" stop-color="#0b1727" />
                <stop offset="100%" stop-color="#081220" />
              </radialGradient>
              
              <g id="pupilComponent">
                <circle class="pupil" r="45" fill="var(--pupil)" />
                <path d="M -30 18 A 36 36 0 0 0 28 18" fill="none" stroke="var(--iris-blue-2)" stroke-width="8" stroke-linecap="round" />
                <circle r="4" cx="34" cy="20" fill="var(--iris-blue-2)" />
                <g class="glints">
                  <circle r="16" cx="-14" cy="-20" fill="#ffffff" opacity=".96" />
                  <circle r="7" cx="9" cy="-8" fill="#ffffff" opacity=".9" />
                </g>
              </g>
              
              <g id="eyeComponent">
                <circle r="96" fill="none" stroke="var(--ring-dark)" stroke-width="2.4" />
                <circle r="86" fill="#0b1727" />
                <circle r="76" fill="none" stroke="var(--iris-blue)" stroke-width="16" />
                <circle r="62" fill="url(#gIris)" />
                <circle r="58" fill="none" stroke="#166ea0" stroke-width="6" opacity=".6" />

                <g class="pupilGroup">
                  <use href="#pupilComponent" />
                </g>

                <circle class="ripple" r="70" fill="none" stroke="#8ad1ff" stroke-width="8" opacity="0" />

                <path class="brow" d="M -55 -125 Q -35 -145 -10 -148 Q 15 -142 35 -128 Q 20 -132 -5 -140 Q -30 -135 -55 -125 Z" fill="#273345" opacity=".8" />
                <rect class="lidTop" x="-110" y="-250" width="220" height="130" rx="60" fill="#000" />
                <rect class="lidBot" x="-110" y="120" width="220" height="130" rx="60" fill="#000" />
              </g>
            </defs>

            <g class="haloGroup">
              <circle class="halo" cx="300" cy="210" r="105" fill="#2ec4ff" filter="url(#blur8)" />
              <circle class="halo" cx="600" cy="210" r="105" fill="#2ec4ff" filter="url(#blur8)" />
            </g>

            <g class="eq" id="eq" transform="translate(450,205)">
              <rect x="-36" y="0" width="10" height="0" rx="5" fill="#93d2ff" />
              <rect x="-18" y="0" width="10" height="0" rx="5" fill="#c9e6ff" />
              <rect x="0" y="0" width="10" height="0" rx="5" fill="#ffffff" />
              <rect x="18" y="0" width="10" height="0" rx="5" fill="#c9e6ff" />
              <rect x="36" y="0" width="10" height="0" rx="5" fill="#93d2ff" />
            </g>

            <g class="dots" transform="translate(450,245)">
              <circle cx="-12" cy="0" r="3" fill="#a8d7ff" />
              <circle cx="0" cy="0" r="3" fill="#e6f3ff" />
              <circle cx="12" cy="0" r="3" fill="#a8d7ff" />
            </g>

            <g id="leftEye" transform="translate(300,210)">
              <use href="#eyeComponent" />
            </g>

            <g id="rightEye" transform="translate(600,210)">
              <use href="#eyeComponent" />
            </g>
          </svg>
        </div>
      </div>
    `;
  }
}

declare global {
  interface HTMLElementTagNameMap {
    'gdm-robot-face': GdmRobotFace;
  }
}