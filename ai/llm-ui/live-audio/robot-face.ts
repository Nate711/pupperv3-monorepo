/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { LitElement, css, html } from 'lit';
import { customElement, property, state } from 'lit/decorators.js';
import { Analyser } from './analyser';

@customElement('gdm-robot-face')
export class GdmRobotFace extends LitElement {
  private inputAnalyser!: Analyser;
  private outputAnalyser!: Analyser;

  @state() private currentState: 'idle' | 'listening' | 'thinking' | 'speaking' | 'muted' = 'idle';
  @state() private blinkClass = '';
  @state() private pingClass = '';
  @state() private gazeX = 0;
  @state() private gazeY = 0;
  private gazeStartTime = Date.now();
  private gazeUpdateInterval!: number;

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
      background: #000000ff;
      contain: layout style paint;
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
      --iris-blue: #00b4ff;
      --iris-blue-2: #00d4ff;
      --iris-core: #0b1727;
      --pupil: #060d13;
      --iris-h: 200;
      --iris-s: 100;
      --iris-l: 58;
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
    .lidTop {
      transform: translateY(0);
    }

    /* Blink animation */
    .blink .lidTop {
      animation: blinkOnce 800ms cubic-bezier(.3, .7, .2, 1) 1;
    }

    .blink .brow {
      animation: browBlink 800ms cubic-bezier(.3, .7, .2, 1) 1;
    }

    @keyframes blinkOnce {
      0% { transform: translateY(0); }
      45% { transform: translateY(370px); }
      55% { transform: translateY(370px); }
      100% { transform: translateY(0); }
    }

    @keyframes browBlink {
      0% { transform: translateY(0); }
      45% { transform: translateY(10px); }
      55% { transform: translateY(10px); }
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
    this.startGazeAnimation();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this.stopAnimation();
    this.stopGazeAnimation();
    if (this.blinkTimer) {
      clearTimeout(this.blinkTimer);
    }
  }

  private startAnimation() {
    // Use setInterval at 2Hz (500ms) instead of requestAnimationFrame at 60fps
    // This reduces CPU usage by ~96% (from 60fps to 2fps)
    this.animationFrameId = window.setInterval(() => {
      if (this.inputAnalyser && this.outputAnalyser) {
        this.inputAnalyser.update();
        this.outputAnalyser.update();

        // Determine state based on audio levels
        const inputLevel = this.inputAnalyser.data[0] / 255;
        const outputLevel = this.outputAnalyser.data[0] / 255;

        if (outputLevel > 0.05) {
          this.setState('speaking');
        } else if (inputLevel > 0.02) {
          this.setState('listening');
        } else {
          this.setState('idle');
        }
      }
    }, 500); // 500ms = 2Hz
  }

  private stopAnimation() {
    if (this.animationFrameId) {
      clearInterval(this.animationFrameId);
      this.animationFrameId = null as any;
    }
    if (this.speakingAnimationFrameId) {
      cancelAnimationFrame(this.speakingAnimationFrameId);
    }
  }

  private setState(newState: typeof this.currentState) {
    if (this.currentState !== newState) {
      this.currentState = newState;
      this.requestUpdate();

      // Emit custom event for mode change
      this.dispatchEvent(new CustomEvent('mode-change', {
        detail: { mode: newState },
        bubbles: true,
        composed: true
      }));
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
      this.blinkTimer = window.setTimeout(() => {
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
    }, 800);
  }

  private updateGaze() {
    const currentTime = Date.now();
    const elapsedTime = (currentTime - this.gazeStartTime) / 1000; // Convert to seconds

    // 0.1Hz frequency means one complete cycle every 10 seconds
    // Use different phases for X and Y to create more natural wandering
    const frequencyX = 0.1;
    const frequencyY = 0.08; // Slightly different frequency for Y

    // Create smooth sinusoidal movement with limited range
    // Range: ±15 pixels for subtle eye movement
    this.gazeX = Math.sin(2 * Math.PI * frequencyX * elapsedTime) * 6;
    this.gazeY = Math.cos(2 * Math.PI * frequencyY * elapsedTime) * 4; // Slightly less vertical movement

    this.requestUpdate();
  }

  private startGazeAnimation() {
    // Update gaze every 250ms (4Hz) for smooth interpolation
    // Reduced frequency to minimize style recalculation
    this.gazeUpdateInterval = window.setInterval(() => {
      if (this.currentState === 'idle' || this.currentState === 'thinking') {
        this.updateGaze();
      }
    }, 250);
  }

  private stopGazeAnimation() {
    if (this.gazeUpdateInterval) {
      clearInterval(this.gazeUpdateInterval);
      this.gazeUpdateInterval = undefined as any;
    }
  }

  protected render() {
    return html`
      <div class="robot-face-container">
        <div class="stage ${this.blinkClass} ${this.pingClass}" data-state="${this.currentState}" 
             style="--gaze-x: ${this.gazeX}px; --gaze-y: ${this.gazeY}px;">
          <svg viewBox="0 0 900 420" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">
            <defs>
              <filter id="blur8">
                <feGaussianBlur stdDeviation="8" />
              </filter>
              <radialGradient id="gIris" cx="35%" cy="35%" r="80%">
                <stop offset="0%" stop-color="#0084ffff" />
                <stop offset="70%" stop-color="#0b1727" />
                <stop offset="100%" stop-color="#081220" />
              </radialGradient>
              
              <g id="pupilComponent">
                <circle class="pupil" r="96" fill="var(--pupil)" />
                <path d="M -62 38 A 74 74 0 0 0 58 38" fill="none" stroke="var(--iris-blue-2)" stroke-width="17" stroke-linecap="round" />
                <circle r="8" cx="70" cy="42" fill="var(--iris-blue-2)" />
                <g class="glints">
                  <circle r="31" cx="-29" cy="-51" fill="#ffffff" opacity=".96" />
                  <circle r="14" cx="18" cy="-17" fill="#ffffff" opacity=".9" />
                </g>
              </g>
              
              <g id="eyeComponent">
                <circle r="126" fill="url(#gIris)" />
                <circle r="126" fill="none" stroke="#00a6ffff" stroke-width="6" opacity=".7" />

                <g class="pupilGroup">
                  <use href="#pupilComponent" />
                </g>

                <rect class="lidTop" x="-228" y="-700" width="456" height="500" rx="126" fill="#000" />
              </g>
              
              <g id="eyebrowComponent">
                <path class="brow" d="M -84 -180 Q 0 -204 84 -180" fill="none" stroke="#b9b9b9ff" stroke-width="10" stroke-linecap="round" opacity=".8" />
              </g>
            </defs>

            <g class="haloGroup">
              <circle class="halo" cx="220" cy="210" r="150" fill="#00b4ff" filter="url(#blur8)" />
              <circle class="halo" cx="680" cy="210" r="150" fill="#00b4ff" filter="url(#blur8)" />
            </g>

            <g id="leftEye" transform="translate(220,210)">
              <use href="#eyeComponent" />
              <use href="#eyebrowComponent" />
            </g>

            <g id="rightEye" transform="translate(680,210)">
              <use href="#eyeComponent" />
              <use href="#eyebrowComponent" />
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