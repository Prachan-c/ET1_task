const http = require('http').createServer(handler); // Create HTTP server with handler function
const fs = require('fs'); // File system module for serving HTML
const io = require('socket.io')(http); // Socket.IO for real-time client communication
const Gpio = require('onoff').Gpio; // GPIO module for Raspberry Pi
const LED = new Gpio(7, 'out'); // GPIO pin 7 as output for LED
// const pushButton = new Gpio(5, 'in', 'both'); // GPIO pin 5 as input for pushbutton (unused)
const { spawn } = require('child_process'); // Child process module to run Python scripts

// Global state for random mode
let randomvalue = 0; // Tracks random checkbox state (0: off, 1: on)
let isRunningRandom = false; // Prevents concurrent random script loops

// Start server on port 8080
http.listen(8080);

function handler(req, res) {
  // Serve index.html from public folder
  fs.readFile(__dirname + '/public/index.html', (err, data) => {
    if (err) {
      res.writeHead(404, { 'Content-Type': 'text/html' });
      return res.end('404 Not Found');
    }
    res.writeHead(200, { 'Content-Type': 'text/html' });
    res.write(data);
    return res.end();
  });
}

function runSensorFusion() {
  const pythonProcess = spawn('python3', ['soc_server.py']);

  pythonProcess.stdout.on('data', (data) => {
    console.log(`[soc_server.py stdout]: ${data.toString()}`);
  });

  pythonProcess.stderr.on('data', (err) => {
    console.error(`[soc_server.py stderr]: ${err.toString()}`);
  });

  pythonProcess.on('close', (code) => {
    console.log(`soc_server.py exited with code ${code}`);
  });

  pythonProcess.on('error', (err) => {
    console.error('Failed to start soc_server.py:', err.message);
  });
}


function runPythonScript(socket, ldata, statusKey, scriptName, emitEvent, direction) {
  /**
   * Run a Python script and emit parsed JSON data to the client.
   *
   * @param {Object} socket - Socket.IO socket for client communication.
   * @param {number} ldata - Checkbox state (0 or 1).
   * @param {string} statusKey - Key for status in JSON (e.g., 'forwardStatus').
   * @param {string} scriptName - Python script to run (e.g., 'move_till_obstacle.py').
   * @param {string} emitEvent - Socket event to emit (e.g., 'forwarddata').
   * @param {string} direction - Movement direction ('Forward' or 'Backward').
   * @returns {Promise<number>} Resolves with the script's exit code.
   */
  return new Promise((resolve, reject) => {
    const statusValue = ldata; // Checkbox state
    console.log(`Running ${scriptName} with ${statusKey}: ${statusValue}`);

    const pythonProcess = spawn('python3', [scriptName]);
    let buffer = ''; // Accumulate stdout data
    pythonProcess.stdout.on('data', (data) => {
      buffer += data.toString();
      console.log(`Data from ${scriptName}:`, data.toString());
      try {
        const jsonMatch = buffer.match(/\{.*?\}/); // Match first JSON object
        if (jsonMatch) {
          const parsedData = JSON.parse(jsonMatch[0]);
          console.log(`Parsed JSON (${scriptName}):`, parsedData);
          parsedData[statusKey] = statusValue; // Add status (e.g., forwardStatus: 1)
          parsedData.distance = parsedData.distance.toFixed(2); // Format distance to 2 decimals
          parsedData.direction = direction; // Add direction (Forward/Backward)
          console.log(`Emitting ${emitEvent}:`, parsedData);
          socket.emit(emitEvent, parsedData); // Emit to client
          buffer = buffer.replace(jsonMatch[0], ''); // Clear processed JSON
        }
      } catch (error) {
        console.error(`JSON parse error (${scriptName}):`, error.message);
      }
    });

    pythonProcess.stderr.on('data', (error) => {
      console.error(`Python error (${scriptName}):`, error.toString());
    });

    pythonProcess.on('close', (code) => {
      console.log(`${scriptName} exited with code ${code}`);
      resolve(code);
    });

    pythonProcess.on('error', (err) => {
      console.error(`Failed to start ${scriptName}:`, err.message);
      reject(err);
    });
  });
}

async function runRandomScripts(socket) {
  /**
   * Run forward and backward Python scripts in a loop until randomvalue is 0.
   *
   * @param {Object} socket - Socket.IO socket for client communication.
   */
  if (isRunningRandom) {
    console.log('Random scripts already running, skipping');
    return;
  }
  isRunningRandom = true;

  try {
    while (randomvalue === 1) {
      // Run forward movement
      await runPythonScript(socket, randomvalue, 'randomStatus', 'move_till_obstacle.py', 'randomdata', 'Forward');
      // Run backward movement
      await runPythonScript(socket, randomvalue, 'randomStatus', 'move_backward.py', 'randomdata', 'Backward');
    }
  } catch (error) {
    console.error('Error in random scripts:', error.message);
  } finally {
    isRunningRandom = false;
    console.log('Random scripts stopped');
  }
}

io.sockets.on('connection', (socket) => {
  // Handle new client connection
  console.log('Client connected:', socket.id);

  // socket.on('forward', async (ldata) => {
  //   // Run forward movement script
  //   console.log(`Received forward ldata: ${ldata}`);
  //   try {
  //     await runPythonScript(socket, ldata, 'forwardStatus', 'move_till_obstacle.py', 'forwarddata', 'Forward');
  //   } catch (error) {
  //     console.error('Error in forward script:', error.message);
  //   }
  // });

  // socket.on('backward', async (ldata) => {
  //   // Run backward movement script, followed by forward
  //   console.log(`Received backward ldata: ${ldata}`);
  //   try {
  //     await runPythonScript(socket, ldata, 'backwardStatus', 'move_backward.py', 'backwarddata', 'Backward');
  //     await runPythonScript(socket, ldata, 'backwardStatus', 'move_till_obstacle.py', 'backwarddata', 'Forward');
  //   } catch (error) {
  //     console.error('Error in backward scripts:', error.message);
  //   }
  // });

  // socket.on('random', (ldata) => {
  //   // Toggle random mode and run scripts if enabled
  //   console.log(`Received random ldata: ${ldata}, current randomvalue: ${randomvalue}`);
  //   randomvalue = ldata;
  //   if (randomvalue === 1) {
  //     runRandomScripts(socket);
  //   }
  // });

  socket.on('fusion', (ldata) => {
    // Toggle random mode and run scripts if enabled
    console.log(`Received fusion ldata: ${ldata}`);
    runSensorFusion();
    
  });

  socket.on('disconnect', () => {
    console.log('Client disconnected:', socket.id);
  });

  socket.on('sensor_data', (data) => {
    console.log('Received from Python:', data);
  });
});

// === Raw TCP Socket Server (to receive fusion data from sensor_server.py) ===
const net = require('net');

const tcpServer = net.createServer((tcpSocket) => {
  console.log('[TCP] Python sensor server connected');

  tcpSocket.on('data', (data) => {
    try {
      const message = JSON.parse(data.toString().trim());
      console.log('[TCP] Fusion data from sensor_server.py:', message);
      io.emit('fusiondata', message); // broadcast to all connected clients
    } catch (err) {
      console.error('[TCP] Failed to parse:', data.toString());
    }
  });

  tcpSocket.on('end', () => {
    console.log('[TCP] Python sensor server disconnected');
  });
});

tcpServer.listen(6000, '127.0.0.1', () => {
  console.log('[TCP] Listening for sensor_server.py on port 6000');
});


process.on('SIGINT', () => {
  // Clean up GPIO resources on Ctrl+C
  LED.writeSync(0); // Turn off LED
  LED.unexport(); // Free LED GPIO
  // pushButton.unexport(); // Free pushbutton GPIO (unused)
  process.exit(); // Exit process
});