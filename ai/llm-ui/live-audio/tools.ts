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
      const wsUrl = `${wsProtocol}//localhost:8008`;

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

async function sendRobotCommand(name: string, args: any = {}): Promise<{ success: boolean; response?: any; error?: string }> {
  try {
    const ws = await initRobotWebSocket();

    if (ws && ws.readyState === WebSocket.OPEN) {
      const msg = JSON.stringify({ name, args });

      // Create a promise to wait for the response
      return new Promise((resolve) => {
        const responseHandler = (event: MessageEvent) => {
          try {
            const response = JSON.parse(event.data);
            console.log(`🤖 [ROBOT] Received response for '${name}':`, response);
            ws.removeEventListener('message', responseHandler);
            resolve({ success: true, response });
          } catch (error) {
            console.error(`🤖 [ROBOT] Failed to parse response for '${name}':`, error);
            ws.removeEventListener('message', responseHandler);
            resolve({ success: false, error: "Failed to parse response" });
          }
        };

        // Set up response listener
        ws.addEventListener('message', responseHandler);

        // Set timeout for response
        setTimeout(() => {
          ws.removeEventListener('message', responseHandler);
          resolve({ success: false, error: "Response timeout" });
        }, 1000);

        // Send the command
        ws.send(msg);
        console.log(`🤖 [ROBOT] Sent command: ${name}`, args);
      });
    } else {
      console.error(`🤖 [ROBOT] Cannot send command '${name}' - WebSocket not connected.`);
      return { success: false, error: "WebSocket not connected" };
    }
  } catch (error) {
    console.error(`🤖 [ROBOT] Failed to send command '${name}' - connection error:`, error);
    return { success: false, error: `Connection error: ${error}` };
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

export const move_robot = {
  name: "move_robot",
  description: `Move the robot with specified velocities. 
  Constraints: max vx=0.75m/s, max vy=0.5m/s, max wz=2.0rad/s. 
  Minimum movement threshold: vx/vy must be 0 or >=0.2m/s to actually move the robot. 
  If the user doesn't specify a speed, choose a medium speed.
  Forward => positive vx. Backward => negative vx. Left => positive vy. Right => negative vy. Turn right => negative wz. Turn left => positive wz.`,
  parameters: {
    type: Type.OBJECT,
    properties: {
      vx: {
        type: Type.NUMBER,
        description: "Linear velocity in x direction (forward/backward) in m/s. Range: 0 or 0.2 to 0.75. Values <0.2 won't move the robot."
      },
      vy: {
        type: Type.NUMBER,
        description: "Linear velocity in y direction (left/right) in m/s. Range: 0 or 0.2 to 0.5. Values <0.2 won't move the robot."
      },
      wz: {
        type: Type.NUMBER,
        description: "Angular velocity around z axis (rotation) in rad/s. Range: -2.0 to 2.0. Values <0.4 won't move the robot."
      }
    },
    required: ["vx", "vy", "wz"]
  }
};

export const get_battery_percentage = {
  name: "get_battery_percentage",
  description: "Get the current battery percentage of the robot",
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
    move_robot,
    get_battery_percentage,
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
      const result = await sendRobotCommand("activate");
      response = {
        result: result.success ? result.response?.message || "Robot activated successfully" : `Failed to activate robot: ${result.error}`
      };
    }
    else if (fc.name === "deactivate_robot") {
      console.log('🤖 [TOOLS] Deactivating robot');
      const result = await sendRobotCommand("deactivate");
      response = {
        result: result.success ? result.response?.message || "Robot deactivated successfully" : `Failed to deactivate robot: ${result.error}`
      };
    }
    else if (fc.name === "move_robot") {
      const vx = fc.args?.vx ?? 0;
      const vy = fc.args?.vy ?? 0;
      const wz = fc.args?.wz ?? 0;

      console.log(`🤖 [TOOLS] Moving robot - vx: ${vx}, vy: ${vy}, wz: ${wz}`);
      const result = await sendRobotCommand("move", { vx, vy, wz });
      response = {
        result: result.success ? result.response?.message || `Robot moving with velocities vx=${vx}, vy=${vy}, wz=${wz}` : `Failed to move robot: ${result.error}`
      };
    }
    else if (fc.name === "get_battery_percentage") {
      console.log('🔋 [TOOLS] Getting battery percentage');
      const result = await sendRobotCommand("get_battery");
      if (result.success && result.response) {
        const batteryInfo = `Battery: ${result.response.battery_percentage}% (${result.response.battery_status})`;
        response = {
          result: batteryInfo
        };
      } else {
        response = {
          result: `Failed to get battery percentage: ${result.error}`
        };
      }
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