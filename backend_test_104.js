const express = require('express');
const http = require('http');
const { Server } = require('socket.io');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

const app = express();
const server = http.createServer(app);
const io = new Server(server, {
  cors: { origin: "*" }
});

// Khởi tạo serial
const port = new SerialPort({
  path: 'COM5',
  baudRate: 115200,
  autoOpen: false
});
const parser = port.pipe(new ReadlineParser({ delimiter: '\r\n' }));

// Mở cổng COM
port.open((err) => {
  if (err) {
    return console.log('❌ Không thể mở cổng COM:', err.message);
  }
  console.log('✅ COM5 đã kết nối');
});

function sendAT(command, wait = 500) {
  return new Promise((resolve) => {
    console.log(`[SEND] ${command}`);
    const responses = [];
    let timeout;

    const onData = (line) => {
      if (line) {
        console.log(`[RECV] ${line}`);
        responses.push(line);
      }
    };

    parser.on('data', onData);

    port.write(command + '\r', (err) => {
      if (err) return console.log('❌ Lỗi khi gửi lệnh AT:', err.message);
    });

    timeout = setTimeout(() => {
      parser.removeListener('data', onData);
      if (responses.length === 0) {
        console.log("❌ Không có phản hồi từ thiết bị (chuỗi rỗng)");
      }
      resolve(responses.join('\n'));
    }, wait + 4000); // tổng 5s
  });
}

async function gpsInit() {
  console.log("\n📡 Khởi động modem...");

  for (let i = 0; i < 3; i++) {
    const res = await sendAT("AT");
    if (res.includes("OK")) break;
    await new Promise(r => setTimeout(r, 1000));
  }

  await sendAT("AT+CGPS=0");
  await new Promise(r => setTimeout(r, 1000));

  const res = await sendAT("AT+CGPS=1");
  if (res.includes("ERROR")) {
    console.log("❌ Don't Turn On GPS, must retry connect.");
    return false;
  }

  console.log("✅ GPS đã bật.");
  return true;
}

async function readGPSLoop() {
  const ok = await gpsInit();
  if (!ok) return;

  while (true) {
    const res = await sendAT("AT+CGPSINFO");
    if (res.includes("+CGPSINFO:")) {
      try {
        const dataLine = res.split('\n').find(line => line.includes("+CGPSINFO:"));
        const coords = dataLine.split(":")[1].trim();
        const fields = coords.split(",");
        const latitude = fields[0]/100;
        const longitude = fields[2]/100;
        const compass = fields[7];

        console.log(`latitude: ${latitude}, longitude: ${longitude}, compass: ${compass}`);

        io.emit("gps_data", {
          latitude: latitude || "--",
          longitude: longitude || "--",
          compass: compass || "--"
        });

        console.log("Emit success", latitude, longitude, compass);
      } catch (e) {
        console.log("⚠️ Lỗi xử lý tọa độ:", e);
      }
    }
    await new Promise(r => setTimeout(r, 1000));
  }
}

io.on('connection', (socket) => {
  console.log('⚡ Client connected:', socket.id);
});

app.get('/', (req, res) => {
  res.send("✅ GPS Server đang chạy và bắn dữ liệu realtime...");
});

server.listen(8000, () => {
  console.log('🚀 GPS Server is running on http://localhost:8000');
  readGPSLoop(); // bắt đầu đọc GPS khi server chạy
});
