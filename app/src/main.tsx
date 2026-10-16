import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import './index.css'
import { ServerAccess } from './components/ServerAccess'

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <ServerAccess />
  </StrictMode>,
)
