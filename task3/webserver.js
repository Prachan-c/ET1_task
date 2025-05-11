var http = require('http').createServer(handler); //require http server, and create server with function handler()
var fs = require('fs'); //require filesystem module
var io = require('socket.io')(http) //require socket.io module and pass the http object (server)
var Gpio = require('onoff').Gpio; //include onoff to interact with the GPIO
var LED = new Gpio(7, 'out'); //use GPIO pin 7 as output
var pushButton = new Gpio(5, 'in', 'both'); //use GPIO pin 5 as input, and 'both' button presses, and releases should be handled

const { spawn } = require('child_process');


http.listen(8080); //listen to port 8080

function handler (req, res) { //create server
  fs.readFile(__dirname + '/public/index.html', function(err, data) { //read file index.html in public folder
    if (err) {
      res.writeHead(404, {'Content-Type': 'text/html'}); //display 404 on error
      return res.end("404 Not Found");
    }
    res.writeHead(200, {'Content-Type': 'text/html'}); //write HTML
    res.write(data); //write data from index.html
    return res.end();
  });
}

io.sockets.on('connection', function (socket) {// WebSocket Connection
  var forwardvalue = 0; //static variable for current status
  // pushButton.watch(function (err, value) { //Watch for hardware interrupts on pushButton
  //   if (err) { //if an error
  //     console.error('There was an error', err); //output error message to console
  //     return;
  //   }
  //   forwardvalue = value;
  //   console.log("push button called light value :" + forwardvalue);
  //   socket.emit('light', forwardvalue); //send button status to client
  // });

  socket.on('forward', function(ldata) { //get light switch status from client
    forwardvalue = ldata;
    console.log("ldata :"+ ldata + " , forwardvalue : " + forwardvalue)
    let pythonProcess = spawn('python3', ['move_till_obstacle.py']);
    let buffer = ''; // Accumulate output
    pythonProcess.stdout.on("data", (data) => {
      buffer += data.toString(); // Append incoming data to buffer
      console.log("Data received:", data.toString()); // Log raw output
      // Try to extract and parse JSON from the buffer
      try {
        // Look for a valid JSON object in the buffer
        const jsonMatch = buffer.match(/\{.*?\}/); // Match first JSON object
        if (jsonMatch) {
          const parsedData = JSON.parse(jsonMatch[0]); // Parse the JSON
          console.log("original Parsed JSON:", parsedData);
          parsedData.forwardStatus = forwardvalue; // Add light switch status
          parsedData.distance = parsedData.distance.toFixed(2); // Round distance to 2 decimal places
          console.log("Modified Parsed JSON:", parsedData);

          socket.emit("forwarddata", parsedData); // Send to client
          // Remove the processed JSON from the buffer
          buffer = buffer.replace(jsonMatch[0], '');
        }
      } catch (error) {
        console.error("Error parsing JSON:", error.message);
      } 
  });

  pythonProcess.stderr.on("data", (error) => {
      console.error("Python Error:", error.toString());
  });

  // Log when the Python process exits
  pythonProcess.on("close", (code) => {
    console.log(`Python process exited with code ${code}`);
  });

  });

  socket.on('backward', function(ldata) { //get light switch status from client
    backwardvalue = ldata;
    console.log("ldata :"+ ldata + " , backwardvalue : " + backwardvalue)
    let pythonProcess = spawn('python3', ['move_backward.py']);
    let buffer = ''; // Accumulate output
    pythonProcess.stdout.on("data", (data) => {
      buffer += data.toString(); // Append incoming data to buffer
      console.log("Data received:", data.toString()); // Log raw output
      // Try to extract and parse JSON from the buffer
      try {
        // Look for a valid JSON object in the buffer
        const jsonMatch = buffer.match(/\{.*?\}/); // Match first JSON object
        if (jsonMatch) {
          const parsedData = JSON.parse(jsonMatch[0]); // Parse the JSON
          console.log("original Parsed JSON:", parsedData);
          parsedData.backwardStatus = backwardvalue; // Add light switch status
          parsedData.distance = parsedData.distance.toFixed(2); // Round distance to 2 decimal places
          parsedData.direction = "Backward"
          console.log("Modified Parsed JSON:", parsedData);

          socket.emit("backwarddata", parsedData); // Send to client
          // Remove the processed JSON from the buffer
          buffer = buffer.replace(jsonMatch[0], '');
        }
      } catch (error) {
        console.error("Error parsing JSON:", error.message);
      }

      
  });

  pythonProcess.stderr.on("data", (error) => {
      console.error("Python Error:", error.toString());
  });

  // Log when the Python process exits
  pythonProcess.on("close", (code) => {
    console.log(`First Python process (move_backward.py) exited with code ${code}`);
  
      // Run the second Python script
    let pythonProcess2 = spawn('python3', ['move_till_obstacle.py']); // Replace with your second script name
    let buffer2 = ''; // Accumulate output for second script
    pythonProcess2.stdout.on('data', (data) => {
      buffer2 += data.toString();
      console.log('Data received from another_script.py:', data.toString());
      try {
        const jsonMatch = buffer2.match(/\{.*?\}/); // Match first JSON object
        if (jsonMatch) {
          const parsedData = JSON.parse(jsonMatch[0]);
          console.log('Original Parsed JSON (another_script.py):', parsedData);
          parsedData.backwardStatus = backwardvalue; // Add backward status
          parsedData.distance = parsedData.distance.toFixed(2) 
          parsedData.direction = "Forward"
          console.log('Modified Parsed JSON (another_script.py):', parsedData);
          socket.emit('backwarddata', parsedData); // Send to client with a different event
          buffer2 = buffer2.replace(jsonMatch[0], ''); // Clear processed JSON
        }
      } catch (error) {
        console.error('Error parsing JSON (another_script.py):', error.message);
      }
    });

    pythonProcess2.stderr.on('data', (error) => {
      console.error('Python Error (another_script.py):', error.toString());
    });

    pythonProcess2.on('close', (code) => {
      console.log(`Second Python process (another_script.py) exited with code ${code}`);
    });
  });
  
    // When the first script finishes, run the second script
    

  });
});

process.on('SIGINT', function () { //on ctrl+c
  LED.writeSync(0); // Turn LED off
  LED.unexport(); // Unexport LED GPIO to free resources
  pushButton.unexport(); // Unexport Button GPIO to free resources
  process.exit(); //exit completely
});
