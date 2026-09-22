import react from '@vitejs/plugin-react'
import { defineConfig } from 'vite'

// https://vite.dev/config/
export default defineConfig({
  // Capacitor loads the bundle from a local app origin, so emitted asset URLs
  // must remain relative. The same output continues to work with vite preview.
  base: './',
  plugins: [react()],
})
