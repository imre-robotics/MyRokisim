
let isRosConnected = false;
var ros = new ROSLIB.Ros({ url : 'ws://localhost:9090' });
ros.on('connection', function() { 
isRosConnected = true;
document.getElementById('status').innerHTML= 'SYS: ONLINE';
document.getElementById('status').style.backgroundColor= '#27ae60';
document.getElementById('status').style.color= '#000';
});
ros.on('error', function() { 
isRosConnected = false;
document.getElementById('status').innerHTML= 'SYS: LOCAL MODE';
document.getElementById('status').style.backgroundColor= '#f39c12';
document.getElementById('status').style.color= '#000';
});
ros.on('close', function() { 
isRosConnected = false;
document.getElementById('status').innerHTML= 'SYS: LOCAL MODE';
document.getElementById('status').style.backgroundColor= '#f39c12';
document.getElementById('status').style.color= '#000';
});

var jointTopic = new ROSLIB.Topic({ ros : ros, name : '/arm_controller/commands', messageType : 'std_msgs/Float64MultiArray' });
var angles = { j1:0, j2:0, j3:0, j4:0, j5:0, j6:0, grip:0, cam:0 };

// KAMERA ÇEVRİMDIŞI MODU (Sinyal Kesici)
const gazeboCam = document.getElementById('gazebo-cam');
const offlineLayer = document.getElementById('cam-offline-layer');
if(gazeboCam && offlineLayer) {
gazeboCam.onload = () => { gazeboCam.style.opacity = 1; offlineLayer.style.display = 'none'; };
gazeboCam.onerror = () => { gazeboCam.style.opacity = 0; offlineLayer.style.display = 'flex'; };
}

// --- 3D KURULUM (GLOBAL SCOPE) ---
THREE.Object3D.DefaultUp.set(0, 0, 1);
const container = document.getElementById('canvas3d-container');
const cW = container.getBoundingClientRect().width || 500;
const cH = container.getBoundingClientRect().height || 400;

// ORTAM (ENVIRONMENT) SEÇİMLERİ
const environments = {
	default: { name: 'Varsayılan', color: 0x1a1a1a },
	lab: { name: 'Laboratuvar', color: 0x0a2540 },
	kitchen: { name: 'Mutfak', color: 0x3d2817 },
	office: { name: 'Çalışma Masası', color: 0x2a2a2a },
	outdoor: { name: 'Açık Alan', color: 0x87ceeb }
};
let currentEnvironment = 'default';

const scene = new THREE.Scene(); scene.background = new THREE.Color(environments[currentEnvironment].color); 
const camera3d = new THREE.PerspectiveCamera(45, cW / cH, 0.1, 100);
camera3d.position.set(1.5, -1.5, 1.2); 
const renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setSize(cW, cH); container.appendChild(renderer.domElement);
const orbitCtrl = new THREE.OrbitControls(camera3d, renderer.domElement);

const grid = new THREE.GridHelper(2, 20, 0x444444, 0x222222);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
scene.add(new THREE.AxesHelper(0.5));

const matGrey = new THREE.MeshPhongMaterial({color: 0x3a3a3a}); 
const matOrange = new THREE.MeshPhongMaterial({color: 0xd35400});

const robotBase = new THREE.Group(); scene.add(robotBase);
const baseMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.05), matGrey); baseMesh.rotation.x = Math.PI / 2; baseMesh.position.z = 0.025; robotBase.add(baseMesh);

const j1P = new THREE.Group(); j1P.position.set(0, 0, 0.05); robotBase.add(j1P);
const l1M = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.07, 0.2), matOrange); l1M.rotation.x = Math.PI / 2; l1M.position.z = 0.1; j1P.add(l1M);

const j2P = new THREE.Group(); j2P.position.set(0, 0, 0.2); j1P.add(j2P);
const l2M = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.4), matGrey); l2M.rotation.x = Math.PI / 2; l2M.position.z = 0.2; j2P.add(l2M);

const j3P = new THREE.Group(); j3P.position.set(0, 0, 0.4); j2P.add(j3P);
const l3M = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.3), matOrange); l3M.rotation.x = Math.PI / 2; l3M.position.z = 0.15; j3P.add(l3M);

const j4P = new THREE.Group(); j4P.position.set(0, 0, 0.3); j3P.add(j4P);
const l4M = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.1), matGrey); l4M.rotation.x = Math.PI / 2; l4M.position.z = 0.05; j4P.add(l4M);

const j5P = new THREE.Group(); j5P.position.set(0, 0, 0.1); j4P.add(j5P);
const l5M = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.1), matOrange); l5M.rotation.x = Math.PI / 2; l5M.position.z = 0.05; j5P.add(l5M);

const j6P = new THREE.Group(); j6P.position.set(0, 0, 0.1); j5P.add(j6P);
const l6M = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.05), matGrey); l6M.rotation.x = Math.PI / 2; l6M.position.z = 0.025; j6P.add(l6M);

const gripBase = new THREE.Group(); gripBase.position.set(0, 0, 0.05); j6P.add(gripBase);

// MEKANIK GRIPPER - Paralel Çene Mekanizması
const gripMountBase = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.02, 0.08), new THREE.MeshPhongMaterial({color: 0x555555})); 
gripMountBase.position.z = 0.04; gripBase.add(gripMountBase);

// SOL ÇENE (Sol finger)
const gripperLeftFinger = new THREE.Group(); gripperLeftFinger.position.set(0, 0, 0.04); gripBase.add(gripperLeftFinger);
const leftFingerMesh = new THREE.Mesh(new THREE.BoxGeometry(0.015, 0.025, 0.06), matOrange);
leftFingerMesh.position.set(-0.025, 0, 0.03);
gripperLeftFinger.add(leftFingerMesh);

// SAĞ ÇENE (Sağ finger)
const gripperRightFinger = new THREE.Group(); gripperRightFinger.position.set(0, 0, 0.04); gripBase.add(gripperRightFinger);
const rightFingerMesh = new THREE.Mesh(new THREE.BoxGeometry(0.015, 0.025, 0.06), matOrange);
rightFingerMesh.position.set(0.025, 0, 0.03);
gripperRightFinger.add(rightFingerMesh);

// GRIPPER KONTROLÜ: angles.grip değeri 0-0.125 aralığında çeneleri açar/kapar
const gripM = { // Dummy object for compatibility
	updateGripperPosition: function(gripValue) {
		// gripValue: 0 = açık, 0.125 = kapalı
		let fingerOffset = 0.04 * (1 - gripValue / 0.125); // Açılırken ayrılır
		gripperLeftFinger.position.x = -fingerOffset;
		gripperRightFinger.position.x = fingerOffset;
	}
};

const light = new THREE.DirectionalLight(0xffffff, 1); light.position.set(5, 5, 10); scene.add(light);
scene.add(new THREE.AmbientLight(0x404040));

let currentEuler = new THREE.Euler();
let currentQuat = new THREE.Quaternion();

function getEndEffectorPos(tempA) {
j1P.rotation.z = tempA.j1; 
j2P.rotation.y = 1.5708 + tempA.j2; 
j3P.rotation.y = tempA.j3; 
j4P.rotation.x = tempA.j4; 
j5P.rotation.y = tempA.j5; 
j6P.rotation.z = tempA.j6;
robotBase.updateMatrixWorld(true);
let pos = new THREE.Vector3();
gripBase.getWorldPosition(pos);
return pos;
}

function solveIK() {
let target = new THREE.Vector3(
parseFloat(document.getElementById('tX').value), 
parseFloat(document.getElementById('tY').value), 
parseFloat(document.getElementById('tZ').value)
);
if(target.length() > 0.95) { 
document.getElementById('ik_status').innerText = "ERR: OUT OF REACH"; 
document.getElementById('ik_status').style.color = "#e74c3c"; 
speakStatus("Error. Coordinates are out of physical reach limits."); 
return; 
}
document.getElementById('ik_status').innerText = "SYS: CALCULATING..."; 
document.getElementById('ik_status').style.color = "#f1c40f";

let tempAngles = { ...angles };
let lr = 0.5; let delta = 0.01;
for(let i=0; i<200; i++) {
let p = getEndEffectorPos(tempAngles);
let err = p.distanceTo(target);
if(err < 0.005) break; 
let grads = {}; let keys = ['j1','j2','j3','j4','j5','j6'];
for(let k of keys) {
let orig = tempAngles[k]; tempAngles[k] += delta;
let newP = getEndEffectorPos(tempAngles);
grads[k] = (newP.distanceTo(target) - err) / delta;
tempAngles[k] = orig;
}
for(let k of keys) {
tempAngles[k] -= lr * grads[k];
if (k === 'j1') tempAngles.j1 = Math.max(-2.8, Math.min(2.8, tempAngles.j1));
else if (k === 'j2') tempAngles.j2 = Math.max(-1.57, Math.min(1.57, tempAngles.j2));
else if (k === 'j3') tempAngles.j3 = Math.max(-2.45, Math.min(2.45, tempAngles.j3));
else if (k === 'j5') tempAngles.j5 = Math.max(-1.57, Math.min(1.57, tempAngles.j5));
else tempAngles[k] = Math.max(-Math.PI, Math.min(Math.PI, tempAngles[k]));
}
}

['j1','j2','j3','j4','j5','j6'].forEach((k, idx) => {
angles[k] = tempAngles[k];
document.getElementById(k+'_s').value = angles[k];
document.getElementById('v'+(idx+1)).innerText = (angles[k] * 180/Math.PI).toFixed(2) + '°';
});
document.getElementById('ik_status').innerText = "SYS: TARGET REACHED"; 
document.getElementById('ik_status').style.color = "#2ecc71";
}

// =========================================================
// [MOD-CHART] ÜÇLÜ ANALİZ VE FİLTRELENMİŞ KİNEMATİK MOTORU
// =========================================================
const modals = {
pos: { el: document.getElementById('pos_modal'), header: document.getElementById('pos_header'), btn: document.getElementById('pos_btn') },
load: { el: document.getElementById('load_modal'), header: document.getElementById('load_header'), btn: document.getElementById('load_btn') },
vel: { el: document.getElementById('vel_modal'), header: document.getElementById('vel_header'), btn: document.getElementById('vel_btn') }
};

// 🛠️ DÜZELTİLMİŞ PENCERE SÜRÜKLEME KONTROLÜ (Hepsi aynı anda çalışır)
let activeModal = null;
let dragOx = 0, dragOy = 0;

function attachDrag(m) {
m.header.onmousedown = (e) => {
activeModal = m.el;
dragOx = e.clientX - activeModal.offsetLeft;
dragOy = e.clientY - activeModal.offsetTop;
activeModal.style.opacity = "0.8";
activeModal.style.zIndex = "10000"; // Tıklananı en üste getir
};
}
attachDrag(modals.pos); attachDrag(modals.load); attachDrag(modals.vel);

document.addEventListener('mousemove', (e) => {
if (activeModal) {
activeModal.style.left = (e.clientX - dragOx) + "px";
activeModal.style.top = (e.clientY - dragOy) + "px";
}
});

document.addEventListener('mouseup', () => {
if (activeModal) {
activeModal.style.opacity = "1";
activeModal.style.zIndex = "9999";
activeModal = null;
}
});

// BUTON KONTROLLERİ
modals.pos.btn.onclick = () => modals.pos.el.style.display = 'flex';
modals.load.btn.onclick = () => modals.load.el.style.display = 'flex';
modals.vel.btn.onclick = () => modals.vel.el.style.display = 'flex';

// 📈 1. KONUM GRAFİKLERİ KURULUMU
const charts = {}; const ids = ['j1','j2','j3','j4','j5','j6'];
ids.forEach((id, i) => {
charts[id] = new Chart(document.getElementById(`chart_${id}`).getContext('2d'), {
type: 'line',
data: { labels: Array(50).fill(''), datasets: [{ label: `J${i+1} Position`, data: Array(50).fill(0), borderColor: '#3498db', borderWidth: 2, pointRadius: 0, fill: true, backgroundColor: 'rgba(52, 152, 219, 0.1)', tension: 0.4 }] },
options: { responsive: true, maintainAspectRatio: false, animation: false, scales: { y: { grid: {color:'#222'}, ticks: {color:'#888', font:{size:9}}, grace:'5%' }, x: { display: false } }, plugins: { legend: { display: false } } }
});
});

// ⚡ 2. YÜK/TORK GRAFİĞİ KURULUMU
const loadChart = new Chart(document.getElementById('load_chart').getContext('2d'), {
type: 'bar',
data: { labels: ['J1 BASE', 'J2 SHLDR', 'J3 ELBOW', 'J4 WRST1', 'J5 WRST2', 'J6 GRIP'], datasets: [{ data: [0,0,0,0,0,0], backgroundColor: '#e67e22', borderRadius: 5 }] },
options: { indexAxis: 'y', responsive: true, maintainAspectRatio: false, animation: false, scales: { x: { min: 0, max: 100, grid: {color:'#333'}, ticks: {color:'#aaa'} }, y: { ticks: {color:'#fff', font:{family:'Consolas', weight:'bold'}} } }, plugins: { legend: { display: false } } }
});

// 🚀 3. TCP HIZ GRAFİĞİ KURULUMU
const velChart = new Chart(document.getElementById('vel_chart').getContext('2d'), {
type: 'line',
data: { 
labels: Array(60).fill(''), 
datasets: [{ 
label: 'TCP Velocity (m/s)', 
data: Array(60).fill(0), 
borderColor: '#9b59b6', 
borderWidth: 3, pointRadius: 0, fill: true, 
backgroundColor: 'rgba(155, 89, 182, 0.2)', 
tension: 0.4 
}] 
},
options: { 
responsive: true, maintainAspectRatio: false, animation: false, 
scales: { 
y: { min: 0, suggestedMax: 1.0, grid: {color:'#333'}, ticks: {color:'#aaa', font:{family:'Consolas'}} }, 
x: { display: false } 
}, 
plugins: { legend: { display: false } } 
}
});

// VERİ GÜNCELLEME DÖNGÜSÜ
let lastA = {j1:0, j2:0, j3:0, j4:0, j5:0, j6:0};
let lastPos = new THREE.Vector3(0, 0, 0);
let smoothedVelocity = 0; // 🛠️ DÜŞÜK GEÇİREN FİLTRE DEĞİŞKENİ

setInterval(() => {
let currentPos = getEndEffectorPos(angles);

// 1. Konum Güncellemesi
if(modals.pos.el.style.display !== 'none') {
ids.forEach(id => { let d = charts[id].data.datasets[0].data; d.push(angles[id] * 180 / Math.PI); d.shift(); charts[id].update(); });
}
// 2. Yük Güncellemesi
if(modals.load.el.style.display !== 'none') {
let r = Math.hypot(currentPos.x, currentPos.y);
let loads = [
Math.abs(angles.j1 - lastA.j1) * 400 + 5,
(r / 0.95) * 85 + Math.abs(angles.j2 - lastA.j2) * 250 + 10,
Math.abs(angles.j3) * 20 + Math.abs(angles.j3 - lastA.j3) * 200 + 8,
Math.abs(angles.j4 - lastA.j4) * 150 + 5,
Math.abs(angles.j5 - lastA.j5) * 150 + 5,
Math.abs(angles.j6 - lastA.j6) * 150 + (angles.grip * 300) + 2
];
loadChart.data.datasets[0].data = loads.map(l => Math.min(100, l));
loadChart.data.datasets[0].backgroundColor = loads.map(l => l > 80 ? '#c0392b' : (l > 50 ? '#f39c12' : '#2ecc71'));
loadChart.update();
}

// 3. 🚀 Hız (Velocity) Güncellemesi (Filtrelenmiş)
if(modals.vel.el.style.display !== 'none') {
let distance = currentPos.distanceTo(lastPos);
let rawVelocity = distance / 0.1; // Ham Hız (m/s)
// 🛠️ LOW-PASS FILTER: Ani şokları emer, grafiği yumuşatır
smoothedVelocity = (smoothedVelocity * 0.7) + (rawVelocity * 0.3);
// 🛠️ DEADZONE: Çok küçük dijital titreşimleri sıfırlar (Motor durduğunda grafik tam sıfıra iner)
if (smoothedVelocity < 0.005) smoothedVelocity = 0;
let vData = velChart.data.datasets[0].data;
vData.push(smoothedVelocity); vData.shift(); 
velChart.update();
}

lastA = {...angles};
lastPos.copy(currentPos);
}, 100);
// =========================================================

// =========================================================
// ANIMATE 3D (DİJİTAL İKİZ DÖNGÜSÜ)
// =========================================================
function animate3D() {
requestAnimationFrame(animate3D);
let currentP = getEndEffectorPos(angles);
gripM.updateGripperPosition(angles.grip); 
gripBase.getWorldQuaternion(currentQuat);
currentEuler.setFromQuaternion(currentQuat);

orbitCtrl.update(); 
renderer.render(scene, camera3d);

document.getElementById('math_x').innerText = `POS_X: ${currentP.x.toFixed(3)}`;
document.getElementById('math_y').innerText = `POS_Y: ${currentP.y.toFixed(3)}`;
document.getElementById('math_z').innerText = `POS_Z: ${currentP.z.toFixed(3)}`;
document.getElementById('math_roll').innerText = `ROLL : ${(currentEuler.x * 180/Math.PI).toFixed(1)}°`;
document.getElementById('math_pitch').innerText = `PITCH: ${(currentEuler.y * 180/Math.PI).toFixed(1)}°`;
document.getElementById('math_yaw').innerText = `YAW : ${(currentEuler.z * 180/Math.PI).toFixed(1)}°`;
}
animate3D(); 

window.addEventListener('resize', () => { 
let curW = container.getBoundingClientRect().width || 500;
let curH = container.getBoundingClientRect().height || 400;
camera3d.aspect = curW / curH; camera3d.updateProjectionMatrix(); renderer.setSize(curW, curH); 
});

// =========================================================
// MEDIAPIPE AI ENTEGRASYONU (HATA KORUMALI - KAMERA OLMASA DA ÇÖKMEZ)
// =========================================================
const videoElement = document.getElementById('input_video');
const canvasElement = document.getElementById('output_canvas');
const canvasCtx = canvasElement.getContext('2d');
const aiToggle = document.getElementById('ai_toggle');
const statusText = document.getElementById('cv_status');

const hands = new Hands({locateFile: (file) => { return `https://cdn.jsdelivr.net/npm/@mediapipe/hands/${file}`; }});
hands.setOptions({ maxNumHands: 2, modelComplexity: 1, minDetectionConfidence: 0.6, minTrackingConfidence: 0.6 });

hands.onResults((results) => {
canvasCtx.save();
canvasCtx.clearRect(0, 0, canvasElement.width, canvasElement.height);
canvasCtx.drawImage(results.image, 0, 0, canvasElement.width, canvasElement.height);

if (results.multiHandLandmarks && results.multiHandLandmarks.length > 0) {
statusText.innerText = "SYS: TRACKING " + results.multiHandLandmarks.length + " HAND(S)"; 
statusText.style.color = "#2ecc71";
for(let i = 0; i < results.multiHandLandmarks.length; i++) {
drawConnectors(canvasCtx, results.multiHandLandmarks[i], HAND_CONNECTIONS, {color: '#00FF00', lineWidth: 2});
drawLandmarks(canvasCtx, results.multiHandLandmarks[i], {color: '#FF0000', lineWidth: 1, radius: 3});
}

if(aiToggle.checked) {
for(let i = 0; i < results.multiHandLandmarks.length; i++) {
const landmarks = results.multiHandLandmarks[i];
const handLabel = results.multiHandedness[i].label; 

if (handLabel === 'Left') {
let target_j1 = (0.5 - landmarks[0].x) * 3.14; 
let target_j2 = (landmarks[0].y - 0.5) * 2.0;
let target_j3 = (landmarks[8].y - landmarks[5].y) * 5.0; 
let target_j4 = (landmarks[12].y - landmarks[9].y) * 5.0;
let target_j5 = (landmarks[16].y - landmarks[13].y) * 5.0;

angles.j1 += (target_j1 - angles.j1) * 0.15; 
angles.j2 += (target_j2 - angles.j2) * 0.15;
angles.j3 += (target_j3 - angles.j3) * 0.15; 
angles.j4 += (target_j4 - angles.j4) * 0.15; 
angles.j5 += (target_j5 - angles.j5) * 0.15;
angles.j2 = Math.max(-1.57, Math.min(1.57, angles.j2)); 
angles.j3 = Math.max(-2.5, Math.min(2.5, angles.j3));
}

if (handLabel === 'Right') {
let pinchDist = Math.hypot(landmarks[8].x - landmarks[4].x, landmarks[8].y - landmarks[4].y);
let target_grip = 0.0;
if (pinchDist < 0.05) target_grip = 0.125; 
else if (pinchDist > 0.15) target_grip = 0.0;
else target_grip = 0.125 - ((pinchDist - 0.05) / 0.10) * 0.125;

angles.grip += (target_grip - angles.grip) * 0.2;
angles.grip = Math.max(0.0, Math.min(0.125, angles.grip));
}
}

['j1','j2','j3','j4','j5'].forEach((k, idx) => { document.getElementById(k+'_s').value = angles[k]; document.getElementById('v'+(idx+1)).innerText = (angles[k] * 180/Math.PI).toFixed(2) + '°'; });
document.getElementById('grip_s').value = angles.grip; document.getElementById('grip_v').innerText = (angles.grip * 1000).toFixed(2) + ' mm';
}
} else { 
statusText.innerText = "SYS: SCANNING FOR HANDS..."; 
statusText.style.color = "#f1c40f"; 
}
canvasCtx.restore();
});

const camera = new Camera(videoElement, { onFrame: async () => { await hands.send({image: videoElement}); }, width: 400, height: 200 });
// KAMERA YOKSA SİSTEMİN ÇÖKMESİNİ ENGELLEYEN HATA KORUMASI
camera.start().catch(err => {
console.error("Camera Initialisation Error:", err);
statusText.innerText = "SYS: CAM OFFLINE / NO DEVICE";
statusText.style.color = "#e74c3c";
aiToggle.disabled = true; // Kamerası yoksa AI butonunu kapat
});

// =========================================================
// ANALOG JOYSTICK MANTIĞI
// =========================================================
const joyZone = document.getElementById('joyZone'); const joyKnob = document.getElementById('joyKnob');
let isDragging = false; let joyDX = 0, joyDY = 0;

joyKnob.addEventListener('pointerdown', (e) => { isDragging = true; joyKnob.setPointerCapture(e.pointerId); joyKnob.style.transition = 'none'; });
joyKnob.addEventListener('pointerup', (e) => { isDragging = false; joyKnob.releasePointerCapture(e.pointerId); joyKnob.style.transition = 'transform 0.1s ease'; joyKnob.style.transform = `translate(0px, 0px)`; joyDX = 0; joyDY = 0; });
joyKnob.addEventListener('pointermove', (e) => {
if(!isDragging) return;
let rect = joyZone.getBoundingClientRect(); let x = e.clientX - (rect.left + rect.width/2); let y = e.clientY - (rect.top + rect.height/2);
let dist = Math.sqrt(x*x + y*y); if(dist > 30) { x = (x/dist) * 30; y = (y/dist) * 30; }
joyKnob.style.transform = `translate(${x}px, ${y}px)`; joyDX = x / 30; joyDY = y / 30; 
});

setInterval(() => {
if(isDragging && !aiToggle.checked && (Math.abs(joyDX) > 0.05 || Math.abs(joyDY) > 0.05)) {
let tX_el = document.getElementById('tX'); let tY_el = document.getElementById('tY');
let target = new THREE.Vector3(parseFloat(tX_el.value) - joyDY * 0.005, parseFloat(tY_el.value) - joyDX * 0.005, parseFloat(document.getElementById('tZ').value));
tX_el.value = target.x.toFixed(3); tY_el.value = target.y.toFixed(3);

let err = getEndEffectorPos(angles).distanceTo(target);
if(err > 0.001) {
let grads = {}; let keys = ['j1','j2','j3','j4','j5','j6'];
for(let k of keys) {
let orig = angles[k]; angles[k] += 0.01;
grads[k] = (getEndEffectorPos(angles).distanceTo(target) - err) / 0.01; angles[k] = orig;
}
for(let k of keys) {
angles[k] -= 0.1 * grads[k]; 
if(k==='j2') angles[k] = Math.max(-1.57, Math.min(1.57, angles[k]));
else if(k==='j3') angles[k] = Math.max(-2.5, Math.min(2.5, angles[k]));
else if(k==='j5') angles[k] = Math.max(-1.57, Math.min(1.57, angles[k]));
else angles[k] = Math.max(-Math.PI, Math.min(Math.PI, angles[k]));
}
keys.forEach((k, idx) => { document.getElementById(k+'_s').value = angles[k]; document.getElementById('v'+(idx+1)).innerText = (angles[k] * 180/Math.PI).toFixed(2) + '°'; });
}
}
}, 30);

function bindSlider(id, val_id, key, multiplier, suffix) {
document.getElementById(id).oninput = function() { 
var aiCheckbox = document.getElementById('ai_toggle');
if(aiCheckbox && aiCheckbox.checked && key !== 'j6' && key !== 'cam') return; // AI açıkken elle bozmayı engelle
angles[key] = parseFloat(this.value); document.getElementById(val_id).innerText = (angles[key] * multiplier).toFixed(2) + suffix; 
};
}
bindSlider('j1_s', 'v1', 'j1', 180/Math.PI, '°'); bindSlider('j2_s', 'v2', 'j2', 180/Math.PI, '°');
bindSlider('j3_s', 'v3', 'j3', 180/Math.PI, '°'); bindSlider('j4_s', 'v4', 'j4', 180/Math.PI, '°');
bindSlider('j5_s', 'v5', 'j5', 180/Math.PI, '°'); bindSlider('j6_s', 'v6', 'j6', 180/Math.PI, '°');
bindSlider('grip_s', 'grip_v', 'grip', 1000, ' mm'); bindSlider('cam_slider', 'cam_val', 'cam', 180/Math.PI, '°');

// ROS'A VERİ GÖNDERME
setInterval(function() {
if(isRosConnected) {
var msg = new ROSLIB.Message({ layout: { dim: [], data_offset: 0 }, data: [angles.j1, angles.j2, angles.j3, angles.j4, angles.j5, angles.j6, angles.grip, angles.cam] });
jointTopic.publish(msg);
}
}, 50);

// =========================================================
// [MOD-JARVIS] SESLİ GERİ BİLDİRİM MOTORU
// =========================================================
const synth = window.speechSynthesis;
let voiceReady = false;

function speakStatus(text) {
if (synth.speaking) synth.cancel(); 
if (text !== '') {
const utterThis = new SpeechSynthesisUtterance(text);
const voices = synth.getVoices();
let selectedVoice = voices.find(voice => voice.name.includes('Google UK English Male') || voice.name.includes('David') || voice.name.includes('Mark') || (voice.lang.startsWith('en-') && voice.name.includes('Male')));
if (!selectedVoice) selectedVoice = voices.find(voice => voice.lang.startsWith('en-'));
if (selectedVoice) utterThis.voice = selectedVoice;
utterThis.pitch = 0.4; utterThis.rate = 1.0; 
synth.speak(utterThis);
}
}

if (speechSynthesis.onvoiceschanged !== undefined) {
speechSynthesis.onvoiceschanged = () => {
if (!voiceReady) {
setTimeout(() => { speakStatus("System online. Welcome, Mustafa Ali Imre. Autonomous control activated."); }, 1000);
voiceReady = true;
}
};
}

// =========================================================
// [MOD-TEACH] ÖĞRET VE TEKRARLA + HAYALET YÖRÜNGE MOTORU
// =========================================================
const recBtn = document.getElementById('rec_btn');
const playBtn = document.getElementById('play_btn');
let recordedPath = []; let isRecording = false; let isPlaying = false; let recordTimer = null; let playTimer = null; let playIndex = 0;

const ghostMaterial = new THREE.LineBasicMaterial({ color: 0x00ffff, linewidth: 2, transparent: true, opacity: 0.8 }); 
let ghostGeometry = new THREE.BufferGeometry();
let ghostLine = new THREE.Line(ghostGeometry, ghostMaterial);
scene.add(ghostLine); 
let ghostPoints = [];

recBtn.addEventListener('click', () => {
if (isPlaying) return; 
if (!isRecording) {
isRecording = true; recordedPath = []; ghostPoints = []; 
scene.remove(ghostLine); if(ghostGeometry) ghostGeometry.dispose();
recBtn.innerText = "⏹️ STOP REC"; recBtn.style.background = "#8e44ad"; 
speakStatus("Recording trajectory."); statusText.innerText = "🔴 SYS: RECORDING..."; statusText.style.color = "#e74c3c";
recordTimer = setInterval(() => {
recordedPath.push({ j1: angles.j1, j2: angles.j2, j3: angles.j3, j4: angles.j4, j5: angles.j5, j6: angles.j6, grip: angles.grip });
let currentPos = getEndEffectorPos(angles);
ghostPoints.push(new THREE.Vector3(currentPos.x, currentPos.y, currentPos.z));
scene.remove(ghostLine);
ghostGeometry = new THREE.BufferGeometry().setFromPoints(ghostPoints);
ghostLine = new THREE.Line(ghostGeometry, ghostMaterial);
scene.add(ghostLine);
}, 100);
} else {
isRecording = false; clearInterval(recordTimer);
recBtn.innerText = "🔴 REC"; recBtn.style.background = "#c0392b";
speakStatus(`Trajectory saved. ${recordedPath.length} frames recorded.`); 
statusText.innerText = `⏹️ SYS: SAVED (${recordedPath.length} FRAMES)`; statusText.style.color = "#f1c40f";
}
});

playBtn.addEventListener('click', () => {
if (isRecording || recordedPath.length === 0) { if(recordedPath.length === 0) speakStatus("Error. No trajectory in memory."); return; }
if (!isPlaying) {
isPlaying = true; playIndex = 0;
playBtn.innerText = "⏸️ STOP PLAY"; playBtn.style.background = "#f39c12"; 
speakStatus("Executing autonomous playback."); statusText.innerText = "▶️ SYS: PLAYING..."; statusText.style.color = "#27ae60";
document.getElementById('ai_toggle').checked = false; 
playTimer = setInterval(() => {
if (playIndex >= recordedPath.length) playIndex = 0; 
let frame = recordedPath[playIndex];
angles.j1 = frame.j1; angles.j2 = frame.j2; angles.j3 = frame.j3; angles.j4 = frame.j4; angles.j5 = frame.j5; angles.j6 = frame.j6; angles.grip = frame.grip;
let keys = ['j1','j2','j3','j4','j5','j6'];
keys.forEach((k, idx) => { document.getElementById(k+'_s').value = angles[k]; document.getElementById('v'+(idx+1)).innerText = (angles[k] * 180/Math.PI).toFixed(2) + '°'; });
document.getElementById('grip_s').value = angles.grip; document.getElementById('grip_v').innerText = (angles.grip * 1000).toFixed(2) + ' mm';
playIndex++;
}, 100);
} else {
isPlaying = false; clearInterval(playTimer);
playBtn.innerText = "▶️ PLAY"; playBtn.style.background = "#27ae60";
speakStatus("Playback stopped."); statusText.innerText = "⏸️ SYS: PLAY STOPPED"; statusText.style.color = "#f1c40f";
}
});

// =========================================================
// [MOD-HYBRID] OTONOM SESLİ ASİSTAN + YAZILI TERMİNAL
// =========================================================
const micBtn = document.getElementById('mic_btn');
const cmdBtn = document.getElementById('cmd_btn');
const cmdInput = document.getElementById('cmd_input');
const memoryMap = { "küp": new THREE.Vector3(0.5, 0.4, 0.1), "daire": new THREE.Vector3(0.5, -0.4, 0.1), "silindir": new THREE.Vector3(0.0, 0.6, 0.1), "koni": new THREE.Vector3(0.0, -0.6, 0.1) };

function lockOnTarget(commandText, isVoice = false) {
if(!commandText) return;
statusText.innerText = (isVoice ? "🎙️ DUYULDU: " : "⌨️ KOMUT: ") + commandText.toUpperCase();
statusText.style.color = "#3498db";
let commandProcessed = false;

if (commandText.match(/uç|üç|işlevci|işlemci|kıskaç|el|tutucu|kavrama/)) {
if (commandText.match(/aç|bırak|sal/)) { angles.grip = 0.0; statusText.innerText = "🛠️ UÇ İŞLEVCİ: AÇILDI"; speakStatus("End effector opened."); commandProcessed = true; } 
else if (commandText.match(/kapat|tut|sık|al/)) { angles.grip = 0.125; statusText.innerText = "🛠️ UÇ İŞLEVCİ: KAPATILDI"; speakStatus("End effector closed."); commandProcessed = true; }
if(commandProcessed) { document.getElementById('grip_s').value = angles.grip; document.getElementById('grip_v').innerText = (angles.grip * 1000).toFixed(2) + ' mm'; return; }
}

const dereceMatch = commandText.match(/(\d+)\s*derece/); 
if (dereceMatch) {
let derece = parseInt(dereceMatch[1]); let radyan = derece * (Math.PI / 180);
if (commandText.includes("sağ")) { angles.j1 -= radyan; statusText.innerText = `🔄 TABAN: SAĞA ${derece}°`; speakStatus(`Base turning right by ${derece} degrees.`); commandProcessed = true; } 
else if (commandText.includes("sol")) { angles.j1 += radyan; statusText.innerText = `🔄 TABAN: SOLA ${derece}°`; speakStatus(`Base turning left by ${derece} degrees.`); commandProcessed = true; } 
else if (commandText.includes("yukarı")) { angles.j2 -= radyan; statusText.innerText = `🔄 GÖVDE: YUKARI ${derece}°`; speakStatus(`Shoulder pitching up by ${derece} degrees.`); commandProcessed = true; } 
else if (commandText.includes("aşağı")) { angles.j2 += radyan; statusText.innerText = `🔄 GÖVDE: AŞAĞI ${derece}°`; speakStatus(`Shoulder pitching down by ${derece} degrees.`); commandProcessed = true; }

if (commandProcessed) {
angles.j1 = Math.max(-Math.PI, Math.min(Math.PI, angles.j1)); angles.j2 = Math.max(-1.57, Math.min(1.57, angles.j2));
document.getElementById('j1_s').value = angles.j1; document.getElementById('v1').innerText = (angles.j1 * 180/Math.PI).toFixed(2) + '°';
document.getElementById('j2_s').value = angles.j2; document.getElementById('v2').innerText = (angles.j2 * 180/Math.PI).toFixed(2) + '°';
document.getElementById('ai_toggle').checked = false; return; 
}
}

for (const [objectName, targetCoords] of Object.entries(memoryMap)) {
if (commandText.includes(objectName)) {
statusText.innerText = "🚀 HEDEF: " + objectName.toUpperCase(); statusText.style.color = "#e67e22";
speakStatus(`${objectName} hedefine yöneliyorum.`); 
document.getElementById('tX').value = targetCoords.x.toFixed(3); document.getElementById('tY').value = targetCoords.y.toFixed(3); document.getElementById('tZ').value = targetCoords.z.toFixed(3);
document.getElementById('ai_toggle').checked = false; solveIK(); commandProcessed = true; break;
}
}

if(!commandProcessed) { statusText.innerText = "❌ KOMUT ANLAŞILAMADI"; statusText.style.color = "#e74c3c"; speakStatus("Command not recognized. Please repeat."); }
}

// =========================================================
// [MOD-SAFETY] ENDÜSTRİYEL GÜVENLİK VE LİMİT KONTROLÖRÜ
// =========================================================
let isSafetyLockActive = false;
setInterval(() => {
let limitBreached = false; let breachedJoint = "";
if (angles.j2 > 1.57) { angles.j2 = 1.57; limitBreached = true; breachedJoint = "Shoulder"; }
if (angles.j2 < -1.57) { angles.j2 = -1.57; limitBreached = true; breachedJoint = "Shoulder"; }
if (angles.j3 > 2.45) { angles.j3 = 2.45; limitBreached = true; breachedJoint = "Elbow"; }
if (angles.j3 < -2.45) { angles.j3 = -2.45; limitBreached = true; breachedJoint = "Elbow"; }
if (angles.j1 > 2.8) { angles.j1 = 2.8; limitBreached = true; breachedJoint = "Base"; }
if (angles.j1 < -2.8) { angles.j1 = -2.8; limitBreached = true; breachedJoint = "Base"; }
if (angles.j5 > 1.57) { angles.j5 = 1.57; limitBreached = true; breachedJoint = "Wrist"; }
if (angles.j5 < -1.57) { angles.j5 = -1.57; limitBreached = true; breachedJoint = "Wrist"; }

if (limitBreached) {
let keys = ['j1','j2','j3','j4','j5','j6'];
keys.forEach((k, idx) => { document.getElementById(k+'_s').value = angles[k]; document.getElementById('v'+(idx+1)).innerText = (angles[k] * 180/Math.PI).toFixed(2) + '°'; });
if (!isSafetyLockActive) {
isSafetyLockActive = true;
document.getElementById('ai_toggle').checked = false;
if (typeof isPlaying !== 'undefined' && isPlaying) document.getElementById('play_btn').click(); 
statusText.innerText = `🚨 E-STOP: ${breachedJoint.toUpperCase()} LIMIT BREACH!`; statusText.style.color = "#c0392b";
speakStatus(`Warning. Critical limit breached at ${breachedJoint} joint. Emergency stop activated.`);
setTimeout(() => { isSafetyLockActive = false; statusText.innerText = "✅ SYS: SAFETY LOCK CLEARED"; statusText.style.color = "#27ae60"; }, 3000);
}
}
}, 100); 

// =========================================================
// [MOD-WORKSPACE] 3B ERİŞİM KÜRESİ GÖRSELLEŞTİRİCİ
// =========================================================
const workspaceBtn = document.getElementById('workspace_btn');
const wsGeometry = new THREE.SphereGeometry(0.95, 32, 24, 0, Math.PI * 2, 0, Math.PI / 2); 
const wsMaterial = new THREE.MeshBasicMaterial({ color: 0x3498db, wireframe: true, transparent: true, opacity: 0.15 });
const workspaceSphere = new THREE.Mesh(wsGeometry, wsMaterial);
workspaceSphere.position.set(0, 0, 0); workspaceSphere.visible = false; workspaceSphere.rotation.x = Math.PI / 2; 
scene.add(workspaceSphere);

let isWorkspaceVisible = false;
workspaceBtn.addEventListener('click', () => {
isWorkspaceVisible = !isWorkspaceVisible;
workspaceSphere.visible = isWorkspaceVisible;
if (isWorkspaceVisible) {
workspaceBtn.innerText = "🌐 WORKSPACE GİZLE"; workspaceBtn.style.background = "#e67e22"; 
speakStatus("Workspace boundary visualization activated."); statusText.innerText = "🌐 SYS: WORKSPACE VISIBLE"; statusText.style.color = "#3498db";
} else {
workspaceBtn.innerText = "🌐 WORKSPACE GÖSTER"; workspaceBtn.style.background = "#2980b9";
speakStatus("Workspace boundary visualization deactivated."); statusText.innerText = "🌐 SYS: WORKSPACE HIDDEN"; statusText.style.color = "#bdc3c7";
}
});

// =========================================================
// [MOD-AXES] HER EKLEM İÇİN YEREL EKSENLER
// =========================================================
const axesBtn = document.getElementById('axes_btn');
let allAxes = [];
const baseAxes = new THREE.AxesHelper(0.8); baseAxes.rotation.x = -Math.PI / 2; baseAxes.position.set(0, 0, 0); baseAxes.visible = false;
scene.add(baseAxes); allAxes.push(baseAxes);

const robotJoints = [j1P, j2P, j3P, j4P, j5P, j6P, gripBase];
robotJoints.forEach(joint => {
if (joint) {
const localAxis = new THREE.AxesHelper(0.3); localAxis.visible = false; 
joint.add(localAxis); allAxes.push(localAxis);
}
}); 

let isAxesVisible = false;
axesBtn.addEventListener('click', () => {
isAxesVisible = !isAxesVisible;
allAxes.forEach(axis => { axis.visible = isAxesVisible; });
if (isAxesVisible) {
axesBtn.innerText = "📌 EKSENLERİ GİZLE"; axesBtn.style.background = "#c0392b"; 
speakStatus("Local kinematic frames activated. Displaying Denavit-Hartenberg parameters."); statusText.innerText = "📌 SYS: KINEMATIC FRAMES VISIBLE"; statusText.style.color = "#9b59b6";
} else {
axesBtn.innerText = "📌 EKSENLERİ GÖSTER (X-Y-Z)"; axesBtn.style.background = "#8e44ad"; 
speakStatus("Kinematic frames deactivated."); statusText.innerText = "📌 SYS: FRAMES HIDDEN"; statusText.style.color = "#bdc3c7";
}
});

// =========================================================
// TERMİNAL (YAZILI) KONTROLÜ
// =========================================================
function executeCommand() { lockOnTarget(cmdInput.value.trim().toLowerCase(), false); cmdInput.value = ""; }
cmdBtn.addEventListener('click', executeCommand);
cmdInput.addEventListener('keypress', function (e) { if (e.key === 'Enter') executeCommand(); });

const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
if (SpeechRecognition) {
const recognition = new SpeechRecognition(); recognition.lang = 'tr-TR'; recognition.continuous = false; recognition.interimResults = false;
micBtn.addEventListener('click', () => { recognition.start(); micBtn.style.background = "#2ecc71"; micBtn.style.borderColor = "#27ae60"; micBtn.innerText = "🎧 DİNLİYOR..."; });
recognition.onresult = (event) => {
const command = event.results[0][0].transcript.trim().toLowerCase();
micBtn.style.background = "#c0392b"; micBtn.style.borderColor = "#e74c3c"; micBtn.innerText = "🎙️ START MIC";
lockOnTarget(command, true); 
};
recognition.onspeechend = () => { recognition.stop(); micBtn.style.background = "#c0392b"; micBtn.style.borderColor = "#e74c3c"; micBtn.innerText = "🎙️ START MIC"; };
recognition.onerror = (event) => { console.log("Ses Hatası: ", event.error); micBtn.style.background = "#c0392b"; micBtn.innerText = "❌ HATA: " + event.error; };
} else {
micBtn.innerText = "❌ DESTEKLENMİYOR"; micBtn.disabled = true;
}




// =========================================================
        // [MOD-SYS] ZAMAN DAMGALI LOG MOTORU VE SCRIPT PARSER
        // =========================================================
        
        // 1. Konsol Sürükleme ve Buton Bağlantısı
        const consoleModal = { el: document.getElementById('console_modal'), header: document.getElementById('console_header'), btn: document.getElementById('console_btn') };
        attachDrag(consoleModal);
        consoleModal.btn.onclick = () => consoleModal.el.style.display = 'flex';

        // 2. Zaman Damgalı Log Fonksiyonu
        function logToConsole(type, msg) {
            const logBox = document.getElementById('log_box');
            const time = new Date().toLocaleTimeString('tr-TR', { hour12: false });
            let color = '#fff';
            if (type === 'INFO') color = '#3498db';
            if (type === 'WARN') color = '#f39c12';
            if (type === 'ERR') color = '#e74c3c';
            if (type === 'CMD') color = '#2ecc71';
            
            const logLine = `<div style="color:${color}; margin-bottom: 4px;">[${time}] [${type}] ${msg}</div>`;
            logBox.innerHTML += logLine;
            logBox.scrollTop = logBox.scrollHeight; // Otomatik en alta kaydır
        }

        // Sistemi açılış logları ile canlandır
        setTimeout(() => logToConsole('INFO', 'RoKiSim 6.1 Kernel Başlatıldı.'), 500);
        setTimeout(() => logToConsole('INFO', 'Kinematik Çözücü Aktif.'), 800);
        setTimeout(() => logToConsole('WARN', 'ROS 2 Haberleşmesi Bekleniyor...'), 1200);

        // 3. Asenkron Bekleme Fonksiyonu (Scriptteki WAIT komutu için)
        const delay = ms => new Promise(res => setTimeout(res, ms));

        // 4. Endüstriyel Macro Script Çözücü (Parser)
        async function runMacroScript() {
            const scriptText = document.getElementById('script_input').value;
            const lines = scriptText.split('\n');
            
            logToConsole('INFO', '--- GÖREV DİZİSİ BAŞLATILDI ---');
            
            for (let i = 0; i < lines.length; i++) {
                // Yorum satırlarını ve boşlukları temizle
                let rawLine = lines[i].split('//')[0].trim();
                if (rawLine === '') continue;

                let cmd = rawLine.toUpperCase().split(/\s+/); // Boşluklara göre böl
                logToConsole('CMD', `SATIR ${i+1}: ${rawLine}`);

                try {
                    // KOMUT: GOTO X Y Z (Ters Kinematik kullanarak hedefe git)
                    if (cmd[0] === 'GOTO' && cmd.length === 4) {
                        document.getElementById('tX').value = parseFloat(cmd[1]);
                        document.getElementById('tY').value = parseFloat(cmd[2]);
                        document.getElementById('tZ').value = parseFloat(cmd[3]);
                        solveIK(); // Zaten kodunda olan IK fonksiyonunu tetikler
                        await delay(2000); // Robotun hedefe gitmesi için fiziksel pay bırak
                    }
                    // KOMUT: WAIT Milisaniye (Sistemi beklet)
                    else if (cmd[0] === 'WAIT' && cmd.length === 2) {
                        await delay(parseInt(cmd[1]));
                    }
                    // KOMUT: GRIP Değer (Kıskacı aç/kapat)
                    else if (cmd[0] === 'GRIP' && cmd.length === 2) {
                        let gripVal = parseFloat(cmd[1]);
                        angles.grip = Math.max(0.0, Math.min(0.125, gripVal));
                        document.getElementById('grip_s').value = angles.grip;
                        document.getElementById('grip_v').innerText = (angles.grip * 1000).toFixed(2) + ' mm';
                        await delay(500); // Kıskacın kapanma süresi
                    }
                    // KOMUT: HOME (Robotu sıfır noktasına çek)
                    else if (cmd[0] === 'HOME') {
                        ['j1','j2','j3','j4','j5','j6'].forEach(k => angles[k] = 0);
                        ['j1','j2','j3'].forEach((k, idx) => { 
                            document.getElementById(k+'_s').value = 0; 
                            document.getElementById('v'+(idx+1)).innerText = '0.00°'; 
                        });
                        angles.grip = 0;
                        document.getElementById('grip_s').value = 0;
                        document.getElementById('grip_v').innerText = '0.00 mm';
                        await delay(2000);
                    }
                    else {
                        logToConsole('ERR', `Tanımsız Komut veya Eksik Parametre: ${cmd[0]}`);
                    }
                } catch (e) {
                    logToConsole('ERR', `Satır ${i+1} İşlenirken Hata: ${e.message}`);
                }
            }
            
            logToConsole('INFO', '--- GÖREV DİZİSİ TAMAMLANDI ---');
        }
        // =========================================================



        // =========================================================
        // [MOD-EXPORT] ENDÜSTRİYEL DATA LOGGER VE CSV ÇIKTISI
        // =========================================================
        let telemetryLogData = [];
        const maxLogLines = 5000; // Hafızayı şişirmemek için son 5000 satırı tutar

        // Veri toplama döngüsü (Fizik motoruyla aynı hızda, her 100ms'de bir)
        setInterval(() => {
            if (!isRosConnected && angles.j1 === 0 && angles.j2 === 0) return; // Sistem tamamen rölantideyken boş veri kaydetme
            
            let timeStamp = new Date().toISOString().split('T')[1].slice(0, -1); // Saat:Dakika:Saniye.Milisaniye
            let currentP = getEndEffectorPos(angles);
            
            // Log satırını oluştur
            let logLine = {
                time: timeStamp,
                j1: (angles.j1 * 180/Math.PI).toFixed(2),
                j2: (angles.j2 * 180/Math.PI).toFixed(2),
                j3: (angles.j3 * 180/Math.PI).toFixed(2),
                tcp_x: currentP.x.toFixed(4),
                tcp_y: currentP.y.toFixed(4),
                tcp_z: currentP.z.toFixed(4),
                velocity: (typeof smoothedVelocity !== 'undefined' ? smoothedVelocity.toFixed(4) : 0),
                grip: (angles.grip * 1000).toFixed(1)
            };

            telemetryLogData.push(logLine);
            if (telemetryLogData.length > maxLogLines) telemetryLogData.shift(); // En eski veriyi sil
            
            document.getElementById('log_count').innerText = `${telemetryLogData.length} KAYIT`;
        }, 100);

        // CSV İndirme Fonksiyonu
        document.getElementById('export_csv_btn').addEventListener('click', () => {
            if (telemetryLogData.length === 0) {
                speakStatus("Error. Data logger is empty.");
                return;
            }

            // CSV Başlıkları
            let csvContent = "data:text/csv;charset=utf-8,";
            csvContent += "TIMESTAMP,J1_DEG,J2_DEG,J3_DEG,TCP_X,TCP_Y,TCP_Z,VELOCITY_MS,GRIP_MM\n";

            // Verileri virgülle ayırarak CSV formatına çevir
            telemetryLogData.forEach(row => {
                let rowStr = `${row.time},${row.j1},${row.j2},${row.j3},${row.tcp_x},${row.tcp_y},${row.tcp_z},${row.velocity},${row.grip}`;
                csvContent += rowStr + "\n";
            });

            // Tarayıcı üzerinden dosyayı indir
            const encodedUri = encodeURI(csvContent);
            const link = document.createElement("a");
            link.setAttribute("href", encodedUri);
            link.setAttribute("download", `RoKiSim_Telemetry_${new Date().getTime()}.csv`);
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
            
            speakStatus("Telemetry data exported successfully.");
        });
        // =========================================================

// =========================================================
// GAZEBO TOGGLE & ORTAM DEĞİŞTİRME
// =========================================================

function toggleGazeboView() {
	const gazeboPanel = document.getElementById('gazebo-panel');
	const toggleBtn = document.getElementById('toggle_gazebo_btn');
	const visualColumn = document.querySelector('.visual-column');
	
	if(gazeboPanel.style.display === 'none') {
		gazeboPanel.style.display = 'flex';
		toggleBtn.innerText = '👁️ CAM-01 GIZLE';
		toggleBtn.style.background = '#c0392b';
		speakStatus("Gazebo camera feed activated.");
	} else {
		gazeboPanel.style.display = 'none';
		toggleBtn.innerText = '👁️ CAM-01 GÖSTER';
		toggleBtn.style.background = '#2980b9';
		speakStatus("Gazebo camera feed deactivated. Digital twin maximized.");
	}
}

function changeEnvironment(envKey) {
	if(environments[envKey]) {
		currentEnvironment = envKey;
		scene.background = new THREE.Color(environments[envKey].color);
		speakStatus(`Environment changed to ${environments[envKey].name}.`);
		console.log(`🌍 Ortam değiştirildi: ${environments[envKey].name}`);
	}
}
