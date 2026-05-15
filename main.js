const { app, BrowserWindow } = require('electron');
const path = require('path');

function createWindow() {
  const win = new BrowserWindow({
    width: 1300,
    height: 850,
    autoHideMenuBar: true,
    webPreferences: {
      preload: path.join(__dirname, 'src/rokisim_gazebo/web/preload.js'),
      nodeIntegration: false,
      contextIsolation: true,
      sandbox: true,
      enableRemoteModule: false,
      webSecurity: true,
    },
  });

  win.webContents.session.setPermissionCheckHandler((webContents, permission) => {
    return permission === 'media';
  });

  win.webContents.session.setPermissionRequestHandler((webContents, permission, callback) => {
    callback(permission === 'media');
  });

  win.loadFile(path.join(__dirname, 'src/rokisim_gazebo/web/index.html'));
}

app.whenReady().then(createWindow);

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});
