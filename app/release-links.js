'use strict';
(async () => {
  const nav = document.getElementById('release-links');
  if (!nav) return;
  try {
    const response = await fetch('./release.json', {cache: 'no-cache'});
    if (!response.ok) throw new Error('Release information unavailable');
    const release = await response.json();
    const tag = 'v' + release.version;
    const links = [
      ['Download for Windows', `${release.repository}/releases/download/${tag}/${release.windows_asset}`, 'windows-download'],
      ['Source', `${release.repository}/tree/${tag}`, 'source-link'],
      [tag, `${release.repository}/releases/tag/${tag}`, 'release-version']
    ];
    if (['localhost', '127.0.0.1'].includes(location.hostname)) {
      links[0] = ['Check for updates', `${release.repository}/releases/latest`, 'windows-download'];
      links.unshift(['Website', release.website, 'website-link']);
    }
    nav.replaceChildren(...links.map(([label, href, className]) => {
      const a = document.createElement('a');
      a.textContent = label; a.href = href; a.className = className;
      a.target = '_blank'; a.rel = 'noopener noreferrer';
      if (className === 'windows-download') a.title = 'Windows 10/11, 64-bit. Unzip and run Launch EM Plotter.cmd. Calculations run on your device.';
      return a;
    }));
  } catch {
    // Keep the working static release-page link if metadata cannot be loaded.
  }
})();
