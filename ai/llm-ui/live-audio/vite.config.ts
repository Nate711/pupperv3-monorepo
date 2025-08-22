import path from 'path';
import { defineConfig, loadEnv } from 'vite';

export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, '.', '');

  // Check if HTTPS should be disabled via environment variable
  const useHttps = env.VITE_USE_HTTPS == 'true';

  return {
    define: {
      'process.env.API_KEY': JSON.stringify(env.GEMINI_API_KEY),
      'process.env.GEMINI_API_KEY': JSON.stringify(env.GEMINI_API_KEY)
    },
    server: {
      ...(useHttps && {
        https: {
          key: './certs/key.pem',
          cert: './certs/cert.pem'
        }
      }),
      host: '0.0.0.0',
      port: 5173
    },
    resolve: {
      alias: {
        '@': path.resolve(__dirname, '.'),
      }
    }
  };
});
