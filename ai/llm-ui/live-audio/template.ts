/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { html, TemplateResult } from 'lit';

function getBatteryColor(percentage: string): string {
  if (percentage === 'N/A') return '#4CAF50';
  const value = parseInt(percentage.replace('%', ''));
  if (value <= 15) return '#f44336'; // Red for critical
  if (value <= 30) return '#ff9800'; // Orange for low
  return '#4CAF50'; // Green for normal
}

export interface TemplateProps {
  selectedModel: string;
  batteryPercentage: string;
  cpuUsage: string;
  onModelChange: (event: Event) => void;
  onReset: () => void;
  onStartRecording: () => void;
  onStopRecording: () => void;
  onToggleInputAnalyzer: () => void;
  onToggleOutputAnalyzer: () => void;
  onToggleConsole: () => void;
  onClearConsole: () => void;
  isRecording: boolean;
  showInputAnalyzer: boolean;
  showOutputAnalyzer: boolean;
  showConsole: boolean;
  consoleLogs: Array<{ timestamp: string, level: string, message: string }>;
  error: string;
  inputNode: GainNode;
  outputNode: GainNode;
  inputTranscriptions: Array<{ text: string, timestamp: number }>;
  outputTranscriptions: Array<{ text: string, timestamp: number }>;
}

export function renderTemplate(props: TemplateProps): TemplateResult {
  return html`
    <div>
      <div class="top-bar">
        <div class="battery-indicator">
          <div class="battery-icon">
            <div class="battery-shell">
              <div class="battery-fill" 
                   style="--battery-level: ${props.batteryPercentage === 'N/A' ? '0' : (parseInt(props.batteryPercentage.replace('%', '')) / 100)};
                          --battery-color: ${getBatteryColor(props.batteryPercentage)}"></div>
            </div>
            <div class="battery-tip"></div>
          </div>
          ${props.batteryPercentage}
        </div>
        <div class="cpu-indicator">
          CPU ${props.cpuUsage}
        </div>

        <div class="control-group">
          <button
            class="control-button"
            @click=${props.onReset}
            ?disabled=${props.isRecording}
            title="Reset Session">
            <svg
              xmlns="http://www.w3.org/2000/svg"
              height="20px"
              viewBox="0 -960 960 960"
              width="20px"
              fill="currentColor">
              <path
                d="M480-160q-134 0-227-93t-93-227q0-134 93-227t227-93q69 0 132 28.5T720-690v-110h80v280H520v-80h168q-32-56-87.5-88T480-720q-100 0-170 70t-70 170q0 100 70 170t170 70q77 0 139-44t87-116h84q-28 106-114 173t-196 67Z" />
            </svg>
          </button>
          
          ${!props.isRecording ? html`
            <button
              class="control-button"
              @click=${props.onStartRecording}
              title="Start Recording">
              <svg
                viewBox="0 0 100 100"
                width="18px"
                height="18px"
                fill="#c80000"
                xmlns="http://www.w3.org/2000/svg">
                <circle cx="50" cy="50" r="50" />
              </svg>
            </button>
          ` : html`
            <button
              class="control-button recording"
              @click=${props.onStopRecording}
              title="Stop Recording">
              <svg
                viewBox="0 0 100 100"
                width="16px"
                height="16px"
                fill="currentColor"
                xmlns="http://www.w3.org/2000/svg">
                <rect x="25" y="25" width="50" height="50" rx="5" />
              </svg>
            </button>
          `}

          <div style="width: 1px; height: 24px; background: rgba(255, 255, 255, 0.2);"></div>

          <button 
            class="viz-toggle ${props.showInputAnalyzer ? 'active' : ''}"
            @click=${props.onToggleInputAnalyzer}
            title="Toggle Input Visualizer">
            In Vis
          </button>
          <button 
            class="viz-toggle ${props.showOutputAnalyzer ? 'active' : ''}"
            @click=${props.onToggleOutputAnalyzer}
            title="Toggle Output Visualizer">
            Out Vis
          </button>
        </div>

        <div class="model-selector">
          <select @change=${props.onModelChange} .value=${props.selectedModel}>
            <option value="gemini-live-2.5-flash-preview">Gemini Live 2.5 Flash</option>
            <option value="gemini-2.5-flash-preview-native-audio-dialog">Gemini 2.5 Flash Native Audio</option>
          </select>
        </div>
      </div>

      ${props.showInputAnalyzer ? html`
        <div class="audio-visualizer">
          <canvas class="visualizer-canvas"></canvas>
        </div>
      ` : ''}

      ${props.showOutputAnalyzer ? html`
        <div class="output-visualizer">
          <canvas class="visualizer-canvas"></canvas>
        </div>
      ` : ''}

      <div id="status"> ${props.error} </div>
      <gdm-robot-face
        .inputNode=${props.inputNode}
        .outputNode=${props.outputNode}></gdm-robot-face>

      <div class="transcriptions-container">
        <div class="transcription-box input-box">
          <div class="transcription-header">🎤 Input</div>
          <div class="transcriptions-scroll input-scroll">
            ${props.inputTranscriptions.map(t => html`
              <div class="transcription">
                <span class="transcription-text">${t.text}</span>
              </div>
            `)}
          </div>
        </div>
        <div class="transcription-box output-box">
          <div class="transcription-header">🤖 Output</div>
          <div class="transcriptions-scroll output-scroll">
            ${props.outputTranscriptions.map(t => html`
              <div class="transcription">
                <span class="transcription-text">${t.text}</span>
              </div>
            `)}
          </div>
        </div>
      </div>

      <button class="console-toggle" @click=${props.onToggleConsole}>
        ${props.showConsole ? 'Hide Console' : 'Show Console'}
      </button>

      <div class="console-panel ${props.showConsole ? 'show' : ''}">
        <div class="console-header">
          <span>Console (${props.consoleLogs.length} logs)</span>
          <button @click=${props.onClearConsole} style="margin-left: 10px; background: none; border: 1px solid #666; color: white; padding: 2px 6px; border-radius: 3px; font-size: 10px; cursor: pointer;">Clear</button>
          <button class="console-close" @click=${props.onToggleConsole}>×</button>
        </div>
        <div class="console-content">
          ${props.consoleLogs.map(log => html`
            <div class="console-log">
              <span class="console-timestamp">${log.timestamp}</span>
              <span class="console-level ${log.level}">${log.level.toUpperCase()}</span>
              <span class="console-message">${log.message}</span>
            </div>
          `)}
        </div>
      </div>
    </div>
  `;
}