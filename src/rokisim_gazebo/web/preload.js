const { contextBridge } = require('electron');

// Minimal preload bridge to keep renderer isolated and secure.
contextBridge.exposeInMainWorld('electronAPI', {
  platform: process.platform,
  appVersion: process.version,
});
