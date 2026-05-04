import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    proxy: {
      "/auth": "http://localhost:8080",
      "/me": "http://localhost:8080",
      "/organizations": "http://localhost:8080",
      "/apiaries": "http://localhost:8080",
      "/hives": "http://localhost:8080",
      "/healthz": "http://localhost:8080"
    }
  }
});
