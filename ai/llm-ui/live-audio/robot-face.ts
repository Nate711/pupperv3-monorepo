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

  @state() private gazeX = 0;
  @state() private gazeY = 0;
  private gazeStartTime = Date.now();
  private gazeUpdateInterval!: number;

  private _outputNode!: AudioNode;
  private _inputNode!: AudioNode;

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

    /* Default state */
    .stage {
      --iris-s: 90;
      --glow-alpha: .12;
    }

    /* Halos */
    .halo {
      opacity: var(--glow-alpha);
      transform-origin: center;
      animation: breathe 7s ease-in-out infinite;
    }

    @keyframes breathe {
      0%, 100% { transform: scale(1); }
      50% { transform: scale(1.02); }
    }


    /* Gaze - will be updated directly via JavaScript */
    .pupil, .glints {
      transform: translate(0px, 0px);
    }

    /* Eyelids */
    .lidTop {
      transform: translateY(0);
      animation: blinkAnimation 15s infinite;
      animation-timing-function: ease-in-out;
    }
    
    /* Right eye blinks slightly offset for more natural look */
    #rightEye .lidTop {
      animation-delay: 0.08s;
    }
    
    .brow {
      animation: browAnimation 15s infinite;
      animation-timing-function: ease-in-out;
    }
    
    /* Blink animation - slower, more relaxed timing */
    @keyframes blinkAnimation {
      0%, 18% { transform: translateY(0); }
      18.5% { transform: translateY(185px); }
      19% { transform: translateY(370px); }
      19.5% { transform: translateY(370px); }
      20% { transform: translateY(370px); }
      20.5% { transform: translateY(185px); }
      21%, 58% { transform: translateY(0); }
      58.5% { transform: translateY(185px); }
      59% { transform: translateY(370px); }
      59.5% { transform: translateY(370px); }
      60% { transform: translateY(370px); }
      60.5% { transform: translateY(185px); }
      61%, 92% { transform: translateY(0); }
      92.5% { transform: translateY(185px); }
      93% { transform: translateY(370px); }
      93.5% { transform: translateY(370px); }
      94% { transform: translateY(370px); }
      94.5% { transform: translateY(185px); }
      95%, 100% { transform: translateY(0); }
    }
    
    @keyframes browAnimation {
      0%, 18% { transform: translateY(0); }
      18.5% { transform: translateY(5px); }
      19% { transform: translateY(10px); }
      19.5% { transform: translateY(10px); }
      20% { transform: translateY(10px); }
      20.5% { transform: translateY(5px); }
      21%, 58% { transform: translateY(0); }
      58.5% { transform: translateY(5px); }
      59% { transform: translateY(10px); }
      59.5% { transform: translateY(10px); }
      60% { transform: translateY(10px); }
      60.5% { transform: translateY(5px); }
      61%, 92% { transform: translateY(0); }
      92.5% { transform: translateY(5px); }
      93% { transform: translateY(10px); }
      93.5% { transform: translateY(10px); }
      94% { transform: translateY(10px); }
      94.5% { transform: translateY(5px); }
      95%, 100% { transform: translateY(0); }
    }






    @media (prefers-reduced-motion: reduce) {
      .halo, .dots circle {
        animation: none !important;
      }
    }
  `;

  connectedCallback() {
    super.connectedCallback();
    this.startGazeAnimation();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this.stopGazeAnimation();
  }









  private updateGaze() {
    const currentTime = Date.now();
    const elapsedTime = (currentTime - this.gazeStartTime) / 1000; // Convert to seconds

    // 0.1Hz frequency means one complete cycle every 10 seconds
    // Use different phases for X and Y to create more natural wandering
    const frequencyX = 0.1;
    const frequencyY = 0.08; // Slightly different frequency for Y

    // Create smooth sinusoidal movement with limited range
    // Range: ±6 pixels for subtle eye movement
    this.gazeX = Math.sin(2 * Math.PI * frequencyX * elapsedTime) * 6;
    this.gazeY = Math.cos(2 * Math.PI * frequencyY * elapsedTime) * 4; // Slightly less vertical movement

    // Update transform directly on elements (more efficient than CSS vars)
    this.updateGazeElements();
  }

  private updateGazeElements() {
    if (!this.shadowRoot) return;
    
    const pupils = this.shadowRoot.querySelectorAll('.pupil');
    const glints = this.shadowRoot.querySelectorAll('.glints');
    
    const transform = `translate(${this.gazeX}px, ${this.gazeY}px)`;
    
    pupils.forEach(pupil => {
      (pupil as HTMLElement).style.transform = transform;
    });
    
    glints.forEach(glint => {
      (glint as HTMLElement).style.transform = transform;
    });
  }

  private startGazeAnimation() {
    // Update gaze every 250ms (4Hz) for smooth interpolation
    // Reduced frequency to minimize style recalculation
    this.gazeUpdateInterval = window.setInterval(() => {
      this.updateGaze();
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
        <div class="stage">
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