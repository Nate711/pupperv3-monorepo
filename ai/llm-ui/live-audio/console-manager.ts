/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

export interface ConsoleLog {
  timestamp: string;
  level: string;
  message: string;
}

export class ConsoleManager {
  private logs: ConsoleLog[] = [];
  private maxLogs = 100;
  private updateCallback?: () => void;

  constructor(updateCallback?: () => void) {
    this.updateCallback = updateCallback;
    this.setupConsoleInterception();
  }

  private setupConsoleInterception() {
    const originalConsole = {
      log: console.log,
      error: console.error,
      warn: console.warn,
      info: console.info
    };

    const addLog = (level: string, ...args: any[]) => {
      const timestamp = new Date().toLocaleTimeString();
      const message = args.map(arg =>
        typeof arg === 'object' ? JSON.stringify(arg, null, 2) : String(arg)
      ).join(' ');

      this.logs = [...this.logs.slice(-(this.maxLogs - 1)), { timestamp, level, message }];
      this.updateCallback?.();
    };

    console.log = (...args: any[]) => {
      originalConsole.log(...args);
      addLog('log', ...args);
    };

    console.error = (...args: any[]) => {
      originalConsole.error(...args);
      addLog('error', ...args);
    };

    console.warn = (...args: any[]) => {
      originalConsole.warn(...args);
      addLog('warn', ...args);
    };

    console.info = (...args: any[]) => {
      originalConsole.info(...args);
      addLog('info', ...args);
    };
  }

  getLogs(): ConsoleLog[] {
    return this.logs;
  }

  clearLogs() {
    this.logs = [];
    this.updateCallback?.();
  }

  getLogCount(): number {
    return this.logs.length;
  }
}