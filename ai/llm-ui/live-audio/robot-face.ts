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
      animation: blinkOnce 230ms cubic-bezier(.3, .7, .2, 1) 1;
    }

    @keyframes blinkOnce {
      0% { transform: translateY(0); }
      45% { transform: translateY(240px); }
      55% { transform: translateY(240px); }
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

        if (outputLevel > 0.05) {
          this.setState('speaking');
        } else if (inputLevel > 0.02) {
          this.setState('listening');
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
        <div class="stage ${this.blinkClass} ${this.pingClass}" data-state="${this.currentState}">
          <svg viewBox="0 0 900 420" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">
            <defs>
              <filter id="blur8">
                <feGaussianBlur stdDeviation="8" />
              </filter>
              <radialGradient id="gIris" cx="35%" cy="35%" r="80%">
                <stop offset="0%" stop-color="#0957a1ff" />
                <stop offset="70%" stop-color="#0b1727" />
                <stop offset="100%" stop-color="#081220" />
              </radialGradient>
              
              <g id="pupilComponent">
                <circle class="pupil" r="80" fill="var(--pupil)" />
                <path d="M -52 32 A 62 62 0 0 0 48 32" fill="none" stroke="var(--iris-blue-2)" stroke-width="14" stroke-linecap="round" />
                <circle r="7" cx="58" cy="35" fill="var(--iris-blue-2)" />
                <g class="glints">
                  <circle r="26" cx="-24" cy="-34" fill="#ffffff" opacity=".96" />
                  <circle r="12" cx="15" cy="-14" fill="#ffffff" opacity=".9" />
                </g>
              </g>
              
              <g id="eyeComponent">
                <circle r="105" fill="url(#gIris)" />
                <circle r="100" fill="none" stroke="#0090dd" stroke-width="5" opacity=".7" />

                <g class="pupilGroup">
                  <use href="#pupilComponent" />
                </g>

                <rect class="lidTop" x="-190" y="-430" width="380" height="225" rx="105" fill="#000" />
              </g>
              
              <g id="eyebrowComponent">
                <rect class="brow" x="-80" y="-160" width="160" height="12" rx="6" fill="#6b6b6b" opacity=".9" />
              </g>
            </defs>

            <g class="haloGroup">
              <circle class="halo" cx="270" cy="210" r="120" fill="#00b4ff" filter="url(#blur8)" />
              <circle class="halo" cx="630" cy="210" r="120" fill="#00b4ff" filter="url(#blur8)" />
            </g>

            <g id="leftEye" transform="translate(270,210)">
              <use href="#eyeComponent" />
              <use href="#eyebrowComponent" />
            </g>

            <g id="rightEye" transform="translate(630,210)">
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