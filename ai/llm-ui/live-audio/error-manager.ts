/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

export class ErrorManager {
  private updateCallback?: (error: string) => void;

  constructor(updateCallback?: (error: string) => void) {
    this.updateCallback = updateCallback;
  }

  handleGoogleAIError(error: any) {
    console.error('Google AI API Error:', error);

    // Check for HTTP status codes
    if (error.status || error.code) {
      const statusCode = error.status || error.code;

      switch (statusCode) {
        case 429:
          console.error('❌ Rate limit exceeded (429): Too many requests');
          this.updateError('Rate limit exceeded. Please wait before trying again.');
          break;
        case 500:
          console.error('❌ Internal server error (500): Google AI service error');
          this.updateError('Google AI service temporarily unavailable.');
          break;
        case 503:
          console.error('❌ Service unavailable (503): Google AI overloaded');
          this.updateError('Google AI service overloaded. Please try again later.');
          break;
        case 401:
          console.error('❌ Unauthorized (401): Invalid API key');
          this.updateError('Invalid API key. Please check your credentials.');
          break;
        case 403:
          console.error('❌ Forbidden (403): Access denied');
          this.updateError('Access denied. Please check your permissions.');
          break;
        case 400:
          console.error('❌ Bad request (400): Invalid request format');
          this.updateError('Invalid request format.');
          break;
        case 502:
          console.error('❌ Bad gateway (502): Upstream server error');
          this.updateError('Google AI gateway error. Please try again.');
          break;
        case 504:
          console.error('❌ Gateway timeout (504): Request timeout');
          this.updateError('Request timeout. Please try again.');
          break;
        default:
          console.error(`❌ HTTP Error (${statusCode}):`, error.message || 'Unknown error');
          this.updateError(`API error (${statusCode}): ${error.message || 'Unknown error'}`);
      }
    } else if (error.message) {
      // Handle generic errors
      console.error('❌ Google AI Error:', error.message);
      this.updateError(`Google AI error: ${error.message}`);
    } else {
      // Handle unknown errors
      console.error('❌ Unknown Google AI Error:', error);
      this.updateError('Unknown Google AI error occurred.');
    }
  }

  private updateError(msg: string) {
    this.updateCallback?.(msg);
  }
}