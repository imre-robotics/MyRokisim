const { app, BrowserWindow } = require('electron');
const path = require('path');

// 1. GÜVENLİK DUVARLARINI KAPAT (Kamera izni sormaması için)
app.commandLine.appendSwitch('ignore-certificate-errors');
app.commandLine.appendSwitch('use-fake-ui-for-media-stream');

function createWindow () {
  const win = new BrowserWindow({
    width: 1300,
    height: 850,
    autoHideMenuBar: true,
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false,
      webSecurity: false // Kamera ve lokal dosyalar (3D motoru) için şart
    }
  });

  // 2. İZİNLERİ OTOMATİK ONAYLA
  win.webContents.session.setPermissionCheckHandler(() => true);
  win.webContents.session.setPermissionRequestHandler((webContents, permission, callback) => {
    callback(true);
  });

  // Altın HTML kodunu yükle
  win.loadFile(path.join(__dirname, 'src/rokisim_gazebo/web/index.html'));
}

app.whenReady().then(createWindow);

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});