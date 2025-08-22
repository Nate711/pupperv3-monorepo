/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { css } from 'lit';

export const styles = css`
  :host {
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
  }

  #status {
    position: absolute;
    bottom: 5vh;
    left: 0;
    right: 0;
    z-index: 10;
    text-align: center;
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
  }

  .top-bar {
    position: absolute;
    top: 20px;
    left: 20px;
    right: 20px;
    z-index: 10;
    display: grid;
    grid-template-columns: 80px 120px 1fr 200px;
    align-items: center;
    gap: 12px;
  }

  .battery-indicator {
    background: rgba(0, 0, 0, 0.7);
    color: white;
    padding: 8px 12px;
    border-radius: 12px;
    font-size: 12px;
    font-weight: 500;
    border: 1px solid rgba(255, 255, 255, 0.2);
    text-align: center;
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }

  .mode-indicator {
    background: rgba(0, 0, 0, 0.7);
    color: white;
    padding: 8px 16px;
    border-radius: 12px;
    font-size: 14px;
    font-weight: 500;
    text-transform: uppercase;
    letter-spacing: 0.5px;
    border: 1px solid rgba(255, 255, 255, 0.2);
    transition: all 0.3s ease;
    text-align: center;
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }

  .mode-indicator.idle {
    background: rgba(100, 100, 100, 0.7);
    border-color: rgba(150, 150, 150, 0.3);
  }

  .mode-indicator.listening {
    background: rgba(25, 162, 230, 0.7);
    border-color: rgba(25, 162, 230, 0.5);
    animation: pulse-listening 0.9s ease-in-out infinite;
  }

  .mode-indicator.thinking {
    background: rgba(128, 90, 213, 0.7);
    border-color: rgba(128, 90, 213, 0.5);
  }

  .mode-indicator.speaking {
    background: rgba(255, 80, 80, 0.7);
    border-color: rgba(255, 80, 80, 0.5);
    animation: pulse-speaking 0.32s ease-in-out infinite;
  }

  .mode-indicator.muted {
    background: rgba(60, 60, 60, 0.7);
    border-color: rgba(80, 80, 80, 0.3);
    filter: grayscale(1);
  }

  @keyframes pulse-listening {
    0%, 100% { transform: scale(1); box-shadow: 0 0 0 0 rgba(25, 162, 230, 0.4); }
    50% { transform: scale(1.05); box-shadow: 0 0 0 10px rgba(25, 162, 230, 0); }
  }

  @keyframes pulse-speaking {
    0%, 100% { transform: scale(1); box-shadow: 0 0 0 0 rgba(255, 80, 80, 0.4); }
    50% { transform: scale(1.03); box-shadow: 0 0 0 8px rgba(255, 80, 80, 0); }
  }

  .control-group {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
    background: rgba(0, 0, 0, 0.7);
    border: 0px solid rgba(255, 255, 255, 0.2);
    border-radius: 12px;
    padding: 6px;
  }

  .control-button {
    background: rgba(255, 255, 255, 0.1);
    color: white;
    border: 1px solid rgba(255, 255, 255, 0.2);
    border-radius: 8px;
    width: 36px;
    height: 36px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    transition: all 0.3s ease;
    padding: 0;
  }

  .control-button:hover:not(:disabled) {
    background: rgba(255, 255, 255, 0.2);
    border-color: rgba(255, 255, 255, 0.3);
  }

  .control-button:disabled {
    opacity: 0.3;
    cursor: not-allowed;
  }

  .control-button.recording {
    background: rgba(200, 0, 0, 0.7);
    border-color: rgba(200, 0, 0, 0.5);
    animation: pulse-recording 1s ease-in-out infinite;
  }

  @keyframes pulse-recording {
    0%, 100% { box-shadow: 0 0 0 0 rgba(200, 0, 0, 0.4); }
    50% { box-shadow: 0 0 0 8px rgba(200, 0, 0, 0); }
  }

  .viz-toggle {
    background: rgba(255, 255, 255, 0.1);
    color: white;
    border: 1px solid rgba(255, 255, 255, 0.2);
    border-radius: 6px;
    padding: 6px 10px;
    font-size: 11px;
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
    cursor: pointer;
    transition: all 0.3s ease;
    white-space: nowrap;
  }

  .viz-toggle:hover {
    background: rgba(255, 255, 255, 0.2);
    border-color: rgba(255, 255, 255, 0.3);
  }

  .viz-toggle.active {
    background: rgba(25, 162, 230, 0.7);
    border-color: rgba(25, 162, 230, 0.5);
  }

  .model-selector {
    flex-shrink: 0;
  }

  .model-selector select {
    background: rgba(0, 0, 0, 0.7);
    color: white;
    border: 1px solid rgba(255, 255, 255, 0.2);
    border-radius: 8px;
    padding: 8px 12px;
    font-size: 14px;
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
    cursor: pointer;
    outline: none;
    transition: all 0.3s ease;
  }

  .model-selector select:hover {
    background: rgba(0, 0, 0, 0.8);
    border-color: rgba(255, 255, 255, 0.3);
  }

  .model-selector select:focus {
    border-color: rgba(25, 162, 230, 0.5);
    box-shadow: 0 0 0 2px rgba(25, 162, 230, 0.2);
  }

  .model-selector option {
    background: #1a1a1a;
    color: white;
    padding: 8px;
  }

  .console-toggle {
    position: absolute;
    bottom: 20px;
    right: 20px;
    z-index: 10;
    background: rgba(0, 0, 0, 0.7);
    color: white;
    border: 1px solid rgba(255, 255, 255, 0.2);
    border-radius: 8px;
    padding: 8px 12px;
    font-size: 12px;
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
    cursor: pointer;
    transition: all 0.3s ease;
  }

  .console-toggle:hover {
    background: rgba(0, 0, 0, 0.8);
    border-color: rgba(255, 255, 255, 0.3);
  }

  .console-panel {
    position: fixed;
    bottom: 0;
    left: 0;
    right: 0;
    height: 30vh;
    background: rgba(0, 0, 0, 0.9);
    border-top: 1px solid rgba(255, 255, 255, 0.2);
    z-index: 20;
    transform: translateY(100%);
    transition: transform 0.3s ease;
    display: flex;
    flex-direction: column;
  }

  .console-panel.show {
    transform: translateY(0);
  }

  .console-header {
    padding: 8px 16px;
    background: rgba(255, 255, 255, 0.1);
    border-bottom: 1px solid rgba(255, 255, 255, 0.2);
    display: flex;
    justify-content: between;
    align-items: center;
    font-size: 12px;
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
    color: #ccc;
  }

  .console-close {
    background: none;
    border: none;
    color: white;
    font-size: 16px;
    cursor: pointer;
    padding: 0;
    margin-left: auto;
  }

  .console-content {
    flex: 1;
    overflow-y: auto;
    padding: 8px;
    font-family: 'Monaco', 'Menlo', 'Ubuntu Mono', monospace;
    font-size: 11px;
    line-height: 1.4;
  }

  .console-log {
    padding: 2px 0;
    border-bottom: 1px solid rgba(255, 255, 255, 0.05);
    display: flex;
    gap: 8px;
  }

  .console-timestamp {
    color: #666;
    flex-shrink: 0;
    width: 60px;
  }

  .console-level {
    flex-shrink: 0;
    width: 50px;
    font-weight: bold;
  }

  .console-level.log {
    color: #fff;
  }

  .console-level.error {
    color: #ff6b6b;
  }

  .console-level.warn {
    color: #ffa726;
  }

  .console-level.info {
    color: #42a5f5;
  }

  .console-message {
    flex: 1;
    color: #ccc;
    word-break: break-word;
  }

  .audio-visualizer {
    position: absolute;
    top: 70px;
    left: 20px;
    z-index: 10;
    width: 200px;
    height: 80px;
    background: rgba(0, 0, 0, 0.7);
    border: 1px solid rgba(255, 255, 255, 0.2);
    border-radius: 8px;
    padding: 8px;
  }

  .visualizer-canvas {
    width: 100%;
    height: 100%;
    background: #000;
    border-radius: 4px;
  }

  .output-visualizer {
    position: absolute;
    top: 70px;
    right: 20px;
    z-index: 10;
    width: 200px;
    height: 80px;
    background: rgba(0, 0, 0, 0.7);
    border: 1px solid rgba(255, 255, 255, 0.2);
    border-radius: 8px;
    padding: 8px;
  }
`;