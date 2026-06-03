// =========================================================
// OTONOM OPC UA KÖPRÜ SUNUCUSU (RoKiSim Backend)
// =========================================================
const express = require('express');
const http = require('http');
const { Server } = require('socket.io');
const opcua = require('node-opcua');
const path = require('path'); // Sadece BİR KERE en tepede tanımladık

const app = express();
const server = http.createServer(app);
const io = new Server(server);

// Web arayüzünün bulunduğu alt klasörü (ROS2 Workspace) sunucuya tam adresle tanıtıyoruz
const webDir = path.join(__dirname, 'src', 'rokisim_gazebo', 'web');
app.use(express.static(webDir)); 

// Garantili Ana Sayfa Yönlendirmesi
app.get('/', (req, res) => {
    res.sendFile(path.join(webDir, 'index.html'));
});

// 1. OPC UA İSTEMCİSİ (CLIENT) KURULUMU
const opcClient = opcua.OPCUAClient.create({
    endpointMustExist: false,
    connectionStrategy: { maxRetry: 3, initialDelay: 2000 }
});
let opcSession = null;

// 2. WEB ARAYÜZÜ İLE HABERLEŞME (WebSockets)
io.on('connection', (socket) => {
    console.log('🟢 YENİ OPERATÖR BAĞLANDI (RoKiSim Frontend)');

    // Arayüzden "Bağlan" emri geldiğinde
    socket.on('connectOPC', async (endpointUrl) => {
        try {
            console.log(`🔌 OPC UA Sunucusuna bağlanılıyor: ${endpointUrl}`);
            await opcClient.connect(endpointUrl);
            opcSession = await opcClient.createSession();
            console.log('✅ OPC UA BAĞLANTISI BAŞARILI!');
            socket.emit('opcStatus', 'ONLINE');
        } catch (err) {
            console.log(`❌ OPC UA BAĞLANTI HATASI: ${err.message}`);
            socket.emit('opcStatus', 'OFFLINE');
        }
    });

    // Arayüzden motor açıları (telemetry) geldiğinde
    socket.on('telemetry', async (data) => {
        if (!opcSession) return; 

        try {
            // TIA Portal / PLC içindeki OPC UA değişkenlerine anlık veri yazma
            const nodesToWrite = [
                { nodeId: "ns=3;s=\"RobotData\".\"J1_Angle\"", attributeId: opcua.AttributeIds.Value, value: { value: { dataType: opcua.DataType.Double, value: data.j1 } } },
                { nodeId: "ns=3;s=\"RobotData\".\"J2_Angle\"", attributeId: opcua.AttributeIds.Value, value: { value: { dataType: opcua.DataType.Double, value: data.j2 } } },
                { nodeId: "ns=3;s=\"RobotData\".\"J3_Angle\"", attributeId: opcua.AttributeIds.Value, value: { value: { dataType: opcua.DataType.Double, value: data.j3 } } },
                { nodeId: "ns=3;s=\"RobotData\".\"J4_Angle\"", attributeId: opcua.AttributeIds.Value, value: { value: { dataType: opcua.DataType.Double, value: data.j4 } } },
                { nodeId: "ns=3;s=\"RobotData\".\"J5_Angle\"", attributeId: opcua.AttributeIds.Value, value: { value: { dataType: opcua.DataType.Double, value: data.j5 } } },
                { nodeId: "ns=3;s=\"RobotData\".\"J6_Angle\"", attributeId: opcua.AttributeIds.Value, value: { value: { dataType: opcua.DataType.Double, value: data.j6 } } }
            ];
            await opcSession.write(nodesToWrite);
        } catch (error) {
            // Hata oluşursa sunucunun çökmemesi için sessizce yakala
        }
    });

    socket.on('disconnect', () => {
        console.log('🔴 OPERATÖR BAĞLANTIYI KESTİ');
    });
});

// SUNUCUYU AYAĞA KALDIR
const PORT = 3000;
server.listen(PORT, () => {
    console.log(`🚀 Sistem Başlatıldı: http://localhost:${PORT}`);
    console.log(`⏳ Endüstriyel bağlantılar bekleniyor...`);
});