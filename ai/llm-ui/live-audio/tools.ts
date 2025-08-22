/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { Type } from '@google/genai';

// WebSocket connection for robot control
let robotWebSocket: WebSocket | null = null;

function initRobotWebSocket(): Promise<WebSocket> {
  return new Promise((resolve, reject) => {
    // If already connected, return existing connection
    if (robotWebSocket && robotWebSocket.readyState === WebSocket.OPEN) {
      resolve(robotWebSocket);
      return;
    }

    // If connecting, wait for it to complete
    if (robotWebSocket && robotWebSocket.readyState === WebSocket.CONNECTING) {
      robotWebSocket.addEventListener("open", () => resolve(robotWebSocket!));
      robotWebSocket.addEventListener("error", reject);
      return;
    }

    try {
      // Use appropriate WebSocket protocol based on current page protocol
      const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
      const wsUrl = `${wsProtocol}//localhost:8765`;
      
      console.log(`🤖 [ROBOT] Attempting to connect to robot server at ${wsUrl}...`);
      robotWebSocket = new WebSocket(wsUrl);
      
      robotWebSocket.addEventListener("open", () => {
        console.log("🤖 [ROBOT] WebSocket connection established");
        resolve(robotWebSocket!);
      });
      
      robotWebSocket.addEventListener("message", (event) => {
        const response = JSON.parse(event.data);
        console.log("🤖 [ROBOT] Response:", response);
      });
      
      robotWebSocket.addEventListener("close", () => {
        console.log("🤖 [ROBOT] WebSocket connection closed");
        robotWebSocket = null;
      });
      
      robotWebSocket.addEventListener("error", (error) => {
        console.error("🤖 [ROBOT] WebSocket connection error - server may not be running:", error);
        robotWebSocket = null;
        reject(error);
      });
    } catch (error) {
      console.error("🤖 [ROBOT] Failed to create WebSocket connection - server may not be running:", error);
      robotWebSocket = null;
      reject(error);
    }
  });
}

async function sendRobotCommand(name: string, args: any = {}): Promise<boolean> {
  try {
    const ws = await initRobotWebSocket();
    
    if (ws && ws.readyState === WebSocket.OPEN) {
      const msg = JSON.stringify({ name, args });
      ws.send(msg);
      console.log(`🤖 [ROBOT] Sent command: ${name}`, args);
      return true;
    } else {
      console.error(`🤖 [ROBOT] Cannot send command '${name}' - WebSocket not connected.`);
      return false;
    }
  } catch (error) {
    console.error(`🤖 [ROBOT] Failed to send command '${name}' - connection error:`, error);
    return false;
  }
}

// Robot control tool definitions
export const activate_robot = {
  name: "activate_robot",
  description: "Activate the robot to start operations",
  parameters: {
    type: Type.OBJECT,
    properties: {}
  }
};

export const deactivate_robot = {
  name: "deactivate_robot", 
  description: "Deactivate the robot to stop operations",
  parameters: {
    type: Type.OBJECT,
    properties: {}
  }
};

// Audio visualizer tool definitions
export const turn_on_audio_visualizers = {
  name: "turn_on_audio_visualizers",
  description: "Turn on the audio visualizers for input and output audio streams",
  parameters: {
    type: Type.OBJECT,
    properties: {
      input: {
        type: Type.BOOLEAN,
        description: "Whether to turn on the input audio visualizer"
      },
      output: {
        type: Type.BOOLEAN,
        description: "Whether to turn on the output audio visualizer"
      }
    }
  }
};

export const turn_off_audio_visualizers = {
  name: "turn_off_audio_visualizers",
  description: "Turn off the audio visualizers for input and output audio streams",
  parameters: {
    type: Type.OBJECT,
    properties: {
      input: {
        type: Type.BOOLEAN,
        description: "Whether to turn off the input audio visualizer"
      },
      output: {
        type: Type.BOOLEAN,
        description: "Whether to turn off the output audio visualizer"
      }
    }
  }
};

export const tools = [{
  functionDeclarations: [
    activate_robot,
    deactivate_robot,
    turn_on_audio_visualizers,
    turn_off_audio_visualizers
  ]
}];

// Tool handler function
export async function handleToolCall(
  toolCall: any,
  toggleInputAnalyzer: () => void,
  toggleOutputAnalyzer: () => void,
  showInputAnalyzer: boolean,
  showOutputAnalyzer: boolean
) {
  console.log('🔧 [TOOLS] Received tool call:', toolCall);
  const functionResponses = [];

  for (const fc of toolCall.functionCalls) {
    let response = { result: "ok" };

    if (fc.name === "activate_robot") {
      console.log('🤖 [TOOLS] Activating robot');
      const success = await sendRobotCommand("activate");
      response = {
        result: success ? "Robot activated successfully" : "Failed to activate robot - connection error"
      };
    }
    else if (fc.name === "deactivate_robot") {
      console.log('🤖 [TOOLS] Deactivating robot');
      const success = await sendRobotCommand("deactivate");
      response = {
        result: success ? "Robot deactivated successfully" : "Failed to deactivate robot - connection error"
      };
    }
    else if (fc.name === "turn_on_audio_visualizers") {
      const input = fc.args?.input ?? true;
      const output = fc.args?.output ?? true;

      console.log(`🎛️ [TOOLS] Turning on visualizers - Input: ${input}, Output: ${output}`);

      // Call existing toggle methods
      if (input && !showInputAnalyzer) {
        toggleInputAnalyzer();
      }
      if (output && !showOutputAnalyzer) {
        toggleOutputAnalyzer();
      }

      response = {
        result: `Audio visualizers turned on - Input: ${input}, Output: ${output}`
      };
    }
    else if (fc.name === "turn_off_audio_visualizers") {
      const input = fc.args?.input ?? true;
      const output = fc.args?.output ?? true;

      console.log(`🎛️ [TOOLS] Turning off visualizers - Input: ${input}, Output: ${output}`);

      // Call existing toggle methods
      if (input && showInputAnalyzer) {
        toggleInputAnalyzer();
      }
      if (output && showOutputAnalyzer) {
        toggleOutputAnalyzer();
      }

      response = {
        result: `Audio visualizers turned off - Input: ${input}, Output: ${output}`
      };
    }

    functionResponses.push({
      id: fc.id,
      name: fc.name,
      response: response
    });
  }

  return functionResponses;
}