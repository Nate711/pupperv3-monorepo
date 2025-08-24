/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import { Type } from '@google/genai';

// WebSocket connection for robot control
let robotWebSocket: WebSocket | null = null;

// Commands that should use debug logging (high-frequency/polling commands)
const DEBUG_LOG_COMMANDS = new Set(['get_battery', 'get_cpu_usage']);

// Request tracking system
interface PendingRequest {
  requestId: string;
  resolve: (value: { success: boolean; response?: any; error?: string }) => void;
  reject: (reason?: any) => void;
  timeoutId: number;
  commandName: string;
}

const pendingRequests = new Map<string, PendingRequest>();

function generateRequestId(): string {
  return `req_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
}

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
        try {
          const response = JSON.parse(event.data);
          const requestId = response.request_id;

          // Determine if this is a debug command based on pending request or response hints
          let isDebugCommand = false;
          let commandName = "unknown";

          if (requestId && pendingRequests.has(requestId)) {
            const pendingRequest = pendingRequests.get(requestId)!;
            commandName = pendingRequest.commandName;
            isDebugCommand = DEBUG_LOG_COMMANDS.has(commandName);

            // Clear timeout and remove from pending
            clearTimeout(pendingRequest.timeoutId);
            pendingRequests.delete(requestId);

            // Log the matched response
            const logMessage = `🤖 [ROBOT] Matched response for request ${requestId} (${commandName})`;
            if (isDebugCommand) {
              console.debug(logMessage, response);
            } else {
              console.log(logMessage, response);
            }

            // Resolve the promise
            pendingRequest.resolve({ success: true, response });
          } else if (requestId) {
            console.warn(`🤖 [ROBOT] Received response for unknown request ID: ${requestId}`);
          } else {
            console.warn(`🤖 [ROBOT] Received response without request ID:`, response);
          }
        } catch (error) {
          console.error("🤖 [ROBOT] Error parsing response:", error);
        }
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


export async function sendRobotCommand(name: string, args: any = {}): Promise<{ success: boolean; response?: any; error?: string }> {
  try {
    const ws = await initRobotWebSocket();

    if (ws && ws.readyState === WebSocket.OPEN) {
      const requestId = generateRequestId();
      const msg = JSON.stringify({ name, args, request_id: requestId });

      // Create a promise to wait for the response
      return new Promise((resolve, reject) => {
        // Set up timeout
        const timeoutId = window.setTimeout(() => {
          pendingRequests.delete(requestId);
          const timeoutMessage = `🤖 [ROBOT] Command '${name}' timed out (request ${requestId})`;
          if (DEBUG_LOG_COMMANDS.has(name)) {
            console.debug(timeoutMessage);
          } else {
            console.warn(timeoutMessage);
          }
          resolve({ success: false, error: "Response timeout" });
        }, 2000); // Increased to 2 seconds

        // Store the pending request
        pendingRequests.set(requestId, {
          requestId,
          resolve,
          reject,
          timeoutId,
          commandName: name
        });

        // Send the command
        ws.send(msg);

        // Use debug logging for high-frequency commands
        const logMessage = `🤖 [ROBOT] Sent command: ${name} (request ${requestId})`;
        if (DEBUG_LOG_COMMANDS.has(name)) {
          console.debug(logMessage, args);
        } else {
          console.log(logMessage, args);
        }
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
  description: "Activate the robot to start operations. If the user says activate, turn on, power on etc call this function.",
  parameters: {
    type: Type.OBJECT,
    properties: {}
  }
};

export const deactivate_robot = {
  name: "deactivate_robot",
  description: "Deactivate the robot to stop operations. If the user says deactivate, power off, turn off etc you should use this function.",
  parameters: {
    type: Type.OBJECT,
    properties: {}
  }
};

export const move_robot = {
  name: "move_robot",
  description: `Move the robot with specified velocities. 
  Constraints: max vx=0.75m/s, max vy=0.5m/s, max wz=2.0rad/s. 
  Minimum movement threshold: vx/vy must be 0 or >=0.4m/s to actually move the robot. 
  If the user doesn't specify a speed, choose a medium speed.
  Forward => positive vx. Backward => negative vx. Left => positive vy. Right => negative vy. Turn right => negative wz. Turn left => positive wz.`,
  parameters: {
    type: Type.OBJECT,
    properties: {
      vx: {
        type: Type.NUMBER,
        description: "Linear velocity in x direction (forward/backward) in m/s. Range: 0 or 0.4 to 0.75. Values <0.4 won't move the robot."
      },
      vy: {
        type: Type.NUMBER,
        description: "Linear velocity in y direction (left/right) in m/s. Range: 0 or 0.4 to 0.5. Values <0.4 won't move the robot."
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

export const get_cpu_usage = {
  name: "get_cpu_usage",
  description: "Get the current CPU usage percentage of the system",
  parameters: {
    type: Type.OBJECT,
    properties: {}
  }
};

export const wait = {
  name: "wait",
  description: "Wait/pause for a specified duration in seconds. Useful for sequencing commands with delays between them.",
  parameters: {
    type: Type.OBJECT,
    properties: {
      duration: {
        type: Type.NUMBER,
        description: "Duration to wait in seconds (0.1 to 60 seconds). Default: 1 second."
      }
    },
    required: []
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
    wait,
    get_battery_percentage,
    get_cpu_usage,
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
    else if (fc.name === "get_cpu_usage") {
      console.log('💻 [TOOLS] Getting CPU usage');
      const result = await sendRobotCommand("get_cpu_usage");
      if (result.success && result.response) {
        const cpuInfo = `CPU Usage: ${result.response.cpu_usage}% (${result.response.system_info?.platform})`;
        response = {
          result: cpuInfo
        };
      } else {
        response = {
          result: `Failed to get CPU usage: ${result.error}`
        };
      }
    }
    else if (fc.name === "wait") {
      const duration = fc.args?.duration ?? 1.0;
      console.log(`⏱️ [TOOLS] Waiting for ${duration} seconds`);
      const result = await sendRobotCommand("wait", { duration });
      response = {
        result: result.success ? result.response?.message || `Waited for ${duration} seconds` : `Failed to wait: ${result.error}`
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