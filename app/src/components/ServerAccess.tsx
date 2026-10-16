import { lazy, Suspense, useEffect, useState } from 'react';
import { clearAccessKeys, restoreAccessKeys, setAccessKeys } from '../services/internal/accessCredentials';

const App = lazy(() => import('../App'));

export function ServerAccess() {
  const [ready, setReady] = useState(false);
  const [loading, setLoading] = useState(true);
  const [operator, setOperator] = useState('');
  const [viewer, setViewer] = useState('');
  const [error, setError] = useState('');
  useEffect(() => {
    let active = true;
    void restoreAccessKeys().then((restored) => {
      if (active) { setReady(restored); setLoading(false); }
    });
    return () => { active = false; };
  }, []);
  if (loading) return <p>Loading server access…</p>;
  if (ready) return <>
    <button style={{ position: 'fixed', top: 8, right: 8, zIndex: 11000 }} onClick={() => {
      clearAccessKeys(); window.location.reload();
    }}>Change access keys</button>
    <Suspense fallback={<p>Connecting…</p>}><App /></Suspense>
  </>;
  return <main style={{ maxWidth: 420, margin: '10vh auto', padding: 24 }}>
    <h1>Connect to Dronetect</h1>
    <p>Enter the access keys provided by the server administrator.</p>
    <form onSubmit={(event) => {
      event.preventDefault();
      try { setAccessKeys({ operator: operator.trim(), viewer: viewer.trim() }); setReady(true); }
      catch { setError('Enter two different 64-character lowercase hexadecimal keys.'); }
    }}>
      <label htmlFor="operator-key">Operator access key</label>
      <input id="operator-key" type="password" autoComplete="off" spellCheck={false} required
        value={operator} onChange={(event) => setOperator(event.target.value)} style={{ width: '100%', margin: '8px 0 20px' }} />
      <label htmlFor="viewer-key">Video viewing key</label>
      <input id="viewer-key" type="password" autoComplete="off" spellCheck={false} required
        value={viewer} onChange={(event) => setViewer(event.target.value)} style={{ width: '100%', margin: '8px 0 20px' }} />
      {error && <p role="alert">{error}</p>}
      <button type="submit">Connect</button>
    </form>
  </main>;
}
